#include "nav_msgs/Odometry.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include "ethzasl_icp_mapper/dynamic_mapper.h"

namespace mapper {

Mapper::Mapper(ros::NodeHandle &n, ros::NodeHandle &pn) :
    n(n),
    pn(pn),
    mapPointCloud(0),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
#if BOOST_VERSION >= 104100
    mapBuildingInProgress(false),
#endif // BOOST_VERSION >= 104100
    useConstMotionModel(getParam<bool>("useConstMotionModel", false)),
    localizing(getParam<bool>("localizing", true)),
    mapping(getParam<bool>("mapping", true)),
    minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
    minMapPointCount(getParam<int>("minMapPointCount", 500)),
    inputQueueSize(getParam<int>("inputQueueSize", 10)),
    minOverlap(getParam<double>("minOverlap", 0.5)),
    maxOverlapToMerge(getParam<double>("maxOverlapToMerge", 0.9)),
    tfRefreshPeriod(getParam<double>("tfRefreshPeriod", 0.01)),
    sensorFrame(getParam<string>("sensor_frame", "")),
    odomFrame(getParam<string>("odom_frame", "odom")),
    mapFrame(getParam<string>("map_frame", "world")),
    tfMapFrame(getParam<string>("tf_map_frame", "map")),
    lidarFrame(getParam<string>("lidar_frame", "lidar")),
    vtkFinalMapName(getParam<string>("vtkFinalMapName", "finalMap.vtk")),
    priorDyn(getParam<double>("priorDyn", 0.5)),
    priorStatic(1. - priorDyn),
    maxAngle(getParam<double>("maxAngle", 0.02)),
    eps_a(getParam<double>("eps_a", 0.05)),
    eps_d(getParam<double>("eps_d", 0.02)),
    alpha(getParam<double>("alpha", 0.99)),
    beta(getParam<double>("beta", 0.99)),
    maxDyn(getParam<double>("maxDyn", 0.95)),
    maxDistNewPoint(pow(getParam<double>("maxDistNewPoint", 0.1), 2)),
    sensorMaxRange(getParam<double>("sensorMaxRange", 80.0)),
    T_localMap_to_map(PM::TransformationParameters::Identity(4, 4)),
    T_scanner_to_map(PM::TransformationParameters::Identity(4, 4)),
    publishStamp(ros::Time::now()),
    tfListener(ros::Duration(30)),
    eps(0.0001),
    cad_trigger(false),
    odom_received(0) {


  // Ensure proper states
  if (localizing == false)
    mapping = false;
  if (mapping == true)
    localizing = true;

  // set logger
  if (getParam<bool>("useROSLogger", false))
    PointMatcherSupport::setLogger(make_shared<PointMatcherSupport::ROSLogger>());

  // Load all parameters stored in external files
  loadExternalParameters();

  PM::Parameters params;
  params["dim"] = "-1";
  params["maxDist"] = toParam(sensorMaxRange);

  radiusFilter = PM::get().DataPointsFilterRegistrar.create(
      "MaxDistDataPointsFilter",
      params);

  // topic initializations
  if (getParam<bool>("subscribe_cloud", true))
    cloudSub = n.subscribe("cloud_in", inputQueueSize, &Mapper::gotCloud, this);
  if (getParam<bool>("subscribe_cad", true))
    cadSub = n.subscribe("cad_interface_node/cad_model",
                         inputQueueSize,
                         &Mapper::gotCAD,
                         this);

  mapPub = n.advertise<sensor_msgs::PointCloud2>("point_map", 2, true);
  scanPub = n.advertise<sensor_msgs::PointCloud2>("corrected_scan", 2, true);
  outlierPub = n.advertise<sensor_msgs::PointCloud2>("outliers", 2, true);
  odomPub = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
  posePub = n.advertise<geometry_msgs::TransformStamped>("icp_pose", 50, true);
  odomErrorPub = n.advertise<nav_msgs::Odometry>("icp_error_odom", 50, true);

  // service initializations
  getPointMapSrv =
      n.advertiseService("dynamic_point_map", &Mapper::getPointMap, this);
  saveMapSrv = pn.advertiseService("save_map", &Mapper::saveMap, this);
  loadMapSrv = pn.advertiseService("load_map", &Mapper::loadMap, this);
  resetSrv = pn.advertiseService("reset", &Mapper::reset, this);
  correctPoseSrv =
      pn.advertiseService("correct_pose", &Mapper::correctPose, this);
  initialTransformSrv =
      pn.advertiseService("intial_transform", &Mapper::initialTransform, this);
  loadPublishedMapSrv = pn.advertiseService("load_published_map",
                                            &Mapper::loadPublishedMap,
                                            this);
  setModeSrv = pn.advertiseService("set_mode", &Mapper::setMode, this);
  getModeSrv = pn.advertiseService("get_mode", &Mapper::getMode, this);
  getBoundedMapSrv =
      pn.advertiseService("get_bounded_map", &Mapper::getBoundedMap, this);
  reloadAllYamlSrv =
      pn.advertiseService("reload_all_yaml", &Mapper::reloadallYaml, this);

  // refreshing tf transform thread
  publishThread =
      boost::thread(boost::bind(&Mapper::publishLoop, this, tfRefreshPeriod));
}

Mapper::~Mapper() {
#if BOOST_VERSION >= 104100
  // wait for map-building thread
  if (mapBuildingInProgress)
  {
      mapBuildingFuture.wait();
      if (mapBuildingFuture.has_value())
          delete mapBuildingFuture.get();
  }
#endif // BOOST_VERSION >= 104100
  // wait for publish thread
  publishThread.join();
  // save point cloud
  if (mapPointCloud) {
    mapPointCloud->save(vtkFinalMapName);
    delete mapPointCloud;
  }
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2 &cloudMsgIn) {
  if (localizing) {
    if (odom_received < 3) {
      try {
        tf::StampedTransform transform;
        tfListener.lookupTransform(tfMapFrame,
                                   lidarFrame,
                                   cloudMsgIn.header.stamp,
                                   transform);
        odom_received++;
      } catch (tf::TransformException ex) {
        ROS_WARN_STREAM("Transformations still initializing.");
        posePub.publish(PointMatcher_ros::eigenMatrixToTransformStamped<float>(
            T_scanner_to_map.inverse(),
            lidarFrame,
            tfMapFrame,
            cloudMsgIn.header.stamp));
        odom_received++;
      }
    } else {
      unique_ptr<DP> cloud
          (new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(
              cloudMsgIn)));

      processCloud(move(cloud),
                   cloudMsgIn.header.frame_id,
                   cloudMsgIn.header.stamp,
                   cloudMsgIn.header.seq);
    }
  }
}

void Mapper::gotCAD(const sensor_msgs::PointCloud2 &cloudMsgIn) {
  if (cad_trigger) {
    ROS_WARN_STREAM("Processing CAD");
    std::cout << "getting cad" << std::endl;
    // Load the cad map as base map.
    localizing = true;
    mapping = true;
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    reset(req, res);
    unique_ptr<DP> cloud
        (new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
    processCloud(move(cloud),
                 cloudMsgIn.header.frame_id,
                 cloudMsgIn.header.stamp,
                 cloudMsgIn.header.seq);
    mapping = false;
    cad_trigger = false;
    publishLock.unlock();
  }
}

void Mapper::processCloud(unique_ptr<DP> newPointCloud,
                          const std::string &scannerFrame,
                          const ros::Time &stamp,
                          uint32_t seq) {

  // If the sensor frame was not set by the user, use default
  if (sensorFrame == "") {
    this->sensorFrame = scannerFrame;
  }

  // if the future has completed, use the new map

  //TODO: investigate if we need that
  processNewMapIfAvailable();

  // IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
  timer t;

  // Convert point cloud
  const size_t goodCount(newPointCloud->features.cols());
  if (goodCount == 0) {
    ROS_ERROR("[ICP] I found no good points in the cloud");
    return;
  }

  // Dimension of the point cloud, important since we handle 2D and 3D
  const int dimp1(newPointCloud->features.rows());

  // This need to be depreciated, there is addTime for those field in pm
  if (!(newPointCloud->descriptorExists("stamps_Msec")
      && newPointCloud->descriptorExists("stamps_sec")
      && newPointCloud->descriptorExists("stamps_nsec"))) {
    const float Msec = round(stamp.sec / 1e6);
    const float sec = round(stamp.sec - Msec * 1e6);
    const float nsec = round(stamp.nsec);

    const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);
    const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
    const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
    newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
    newPointCloud->addDescriptor("stamps_sec", desc_sec);
    newPointCloud->addDescriptor("stamps_nsec", desc_nsec);

    //cout << "Adding time" << endl;

  }

  // Ensure a minimum amount of point before filtering
  int ptsCount = newPointCloud->getNbPoints();
  if (ptsCount < minReadingPointCount) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << ptsCount
                                                          << " pts.");
    return;
  }

  {

    // Apply filters to incoming cloud, in scanner coordinates
    inputFilters.apply(*newPointCloud);
  }

  string reason;

  try {
    T_scanner_to_map = PointMatcher_ros::eigenMatrixToDim<float>(
        PointMatcher_ros::transformListenerToEigenMatrix<float>(
            tfListener,
            tfMapFrame, // to
            scannerFrame, // from
            stamp
        ), dimp1);
  }
  catch (tf::ExtrapolationException e) {
    ROS_ERROR_STREAM("Extrapolation Exception. stamp = " << stamp << " now = "
                                                         << ros::Time::now()
                                                         << " delta = "
                                                         << ros::Time::now()
                                                             - stamp << endl
                                                         << e.what());
    return;
  }
  catch (...) {
    // everything else
    ROS_ERROR_STREAM("Unexpected exception... ignoring scan X");
    return;
  }
  // Recuring need to see those transformations...
  ROS_DEBUG_STREAM(
      "[ICP] T_scanner_to_map (" << scannerFrame << " to " << mapFrame << "):\n"
                                 << T_scanner_to_map);

  const PM::TransformationParameters T_scanner_to_localMap =
      transformation->correctParameters(
          T_localMap_to_map.inverse() * T_scanner_to_map);

  // Ensure a minimum amount of point after filtering
  ptsCount = newPointCloud->getNbPoints();
  if (ptsCount < minReadingPointCount) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << ptsCount
                                                          << " pts.");
    return;
  }

  // Initialize the map if empty
  if (!icp.hasMap()) {
    ROS_INFO_STREAM("[MAP] Creating an initial map");
    mapCreationTime = stamp;
    if (cad_trigger) {
      ROS_INFO_STREAM(
          "[ICP] T_scanner_to_map (" << scannerFrame << " to " << tfMapFrame
                                     << "):\n" << T_scanner_to_map);
      setMap(updateMap(newPointCloud.release(), T_scanner_to_map, false));
    } else {
      setMap(updateMap(newPointCloud.release(), T_scanner_to_map, false));
    }
    // we must not delete newPointCloud because we just stored it in the mapPointCloud
    return;
  }

  // Check dimension
  if (newPointCloud->getEuclideanDim()
      != icp.getPrefilteredInternalMap().getEuclideanDim()) {
    ROS_ERROR_STREAM("[ICP] Dimensionality missmatch: incoming cloud is "
                         << newPointCloud->getEuclideanDim() << " while map is "
                         << icp.getPrefilteredInternalMap().getEuclideanDim());
    return;
  }

  try {
    // Apply ICP
    PM::TransformationParameters T_updatedScanner_to_map;
    PM::TransformationParameters T_updatedScanner_to_localMap;

    ROS_DEBUG_STREAM(
        "[ICP] Computing - reading: " << newPointCloud->getNbPoints()
                                      << ", reference: "
                                      << icp.getInternalMap().getNbPoints());

    icpMapLock.lock();
    T_updatedScanner_to_localMap = icp(*newPointCloud, T_scanner_to_localMap);
    icpMapLock.unlock();

    T_updatedScanner_to_map = T_localMap_to_map * T_updatedScanner_to_localMap;

    ROS_DEBUG_STREAM(
        "[ICP] T_updatedScanner_to_map:\n" << T_updatedScanner_to_map);
    ROS_DEBUG_STREAM("[ICP] T_updatedScanner_to_localMap:\n"
                         << T_updatedScanner_to_localMap);

    // Ensure minimum overlap between scans
    const double estimatedOverlap = icp.errorMinimizer->getOverlap();
    ROS_DEBUG_STREAM("[ICP] Overlap: " << estimatedOverlap);
    if (estimatedOverlap < minOverlap) {
      ROS_ERROR_STREAM(
          "[ICP] Estimated overlap too small, ignoring ICP correction!");
      return;
    }

    // Compute tf
    publishStamp = stamp;

    // Publish odometry
    if (odomPub.getNumSubscribers()) {
      // Not sure that the transformation represents the odometry
      odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(
          T_updatedScanner_to_map,
          tfMapFrame,
          stamp));
    }
    // Publish pose
    if (posePub.getNumSubscribers()) {
      // Not sure that the transformation represents the odometry
      posePub.publish(PointMatcher_ros::eigenMatrixToTransformStamped<float>(
          T_updatedScanner_to_map,
          lidarFrame,
          tfMapFrame,
          stamp));
    }
    if (mapPub.getNumSubscribers()) {
      mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud,
                                                                        tfMapFrame,
                                                                        mapCreationTime));
    }
    // Publish the corrected scan point cloud
    DP pc = transformation->compute(*newPointCloud, T_updatedScanner_to_map);
    mapPostFilters.apply(pc);
    publishLock.lock();
    if (scanPub.getNumSubscribers() && localizing) {
      ROS_DEBUG_STREAM(
          "Corrected scan publishing " << pc.getNbPoints() << " points");
      scanPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pc,
                                                                         tfMapFrame,
                                                                         stamp));
    }
    publishLock.unlock();


    // check if news points should be added to the map
    if (
        ((estimatedOverlap < maxOverlapToMerge)
            || (icp.getPrefilteredInternalMap().features.cols()
                < minMapPointCount)) && (!mapBuildingInProgress)
        ) {
      // make sure we process the last available map
      mapCreationTime = stamp;

      ROS_DEBUG_STREAM("[MAP] Adding new points in a separate thread");

      mapBuildingTask = MapBuildingTask(boost::bind(&Mapper::updateMap,
                                                    this,
                                                    newPointCloud.release(),
                                                    T_updatedScanner_to_map,
                                                    true));
      mapBuildingFuture = mapBuildingTask.get_future();
      mapBuildingThread =
          boost::thread(boost::move(boost::ref(mapBuildingTask)));
      mapBuildingThread.detach(); // We don't care about joining this one
      sched_yield();
      mapBuildingInProgress = true;
    } else {
      cerr << "SKIPPING MAP" << endl;
      cerr << "estimatedOverlap < maxOverlapToMerge: "
           << (estimatedOverlap < maxOverlapToMerge) << endl;
      cerr << "(icp.getInternalMap().features.cols() < minMapPointCount): "
           << icp.getInternalMap().features.cols() << " < " << minMapPointCount
           << " = " << (icp.getInternalMap().features.cols() < minMapPointCount)
           << endl;
      cerr << "mapBuildingInProgress: " << mapBuildingInProgress << endl;

      bool stateLock = publishLock.try_lock();
      if (stateLock) publishLock.unlock();
      cerr << "publishLock.try_lock(): " << stateLock << endl;

      stateLock = icpMapLock.try_lock();
      if (stateLock) icpMapLock.unlock();
      cerr << "icpMapLock.try_lock(): " << stateLock << endl;

      cerr << "mapBuildingFuture.has_value(): " << mapBuildingFuture.has_value()
           << endl;

    }

  }
  catch (PM::ConvergenceError error) {
    icpMapLock.unlock();
    ROS_ERROR_STREAM("[ICP] failed to converge: " << error.what());
    newPointCloud->save("error_read.vtk");
    icp.getPrefilteredMap().save("error_ref.vtk");
    return;
  }
  catch (...) {
    // everything else
    publishLock.unlock();
    ROS_ERROR_STREAM("Unen XZ");
    return;
  }
  //Statistics about time and real-time capability
  int realTimeRatio =
      100 * t.elapsed() / (stamp.toSec() - lastPoinCloudTime.toSec());
  realTimeRatio *= seq - lastPointCloudSeq;

  ROS_DEBUG_STREAM("[TIME] Total ICP took: " << t.elapsed() << " [s]");
  if (realTimeRatio < 80)
    ROS_INFO_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
  else
    ROS_WARN_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

  lastPoinCloudTime = stamp;
  lastPointCloudSeq = seq;
}

void Mapper::processNewMapIfAvailable() {
#if BOOST_VERSION >= 104100
  if (mapBuildingInProgress && mapBuildingFuture.has_value())
  {
      ROS_DEBUG_STREAM("[MAP] Computation in thread done");
      setMap(mapBuildingFuture.get());
      mapBuildingInProgress = false;
  }
#endif // BOOST_VERSION >= 104100
}

void Mapper::setMap(DP *newMapPointCloud) {

  // delete old map
  if (mapPointCloud && mapPointCloud != newMapPointCloud)
    delete mapPointCloud;

  // set new map
  mapPointCloud = newMapPointCloud;

  // update ICP map
  updateIcpMap(mapPointCloud);

  // Publish map point cloud
  // FIXME this crash when used without descriptor
  publishLock.lock();
  if (mapPub.getNumSubscribers() && mapping) {
    ROS_DEBUG_STREAM(
        "[MAP] publishing " << mapPointCloud->getNbPoints() << " points");
    mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud,
                                                                      tfMapFrame,
                                                                      mapCreationTime));
  }
  publishLock.unlock();
}

void Mapper::updateIcpMap(const DP *newMapPointCloud) {
  ros::Time time_current = ros::Time::now();
  //TODO check if T_odom_to_scanner can be reused
  try {

    // Move the global map to the scanner pose
    T_scanner_to_map =
        PointMatcher_ros::eigenMatrixToDim<float>(
            PointMatcher_ros::transformListenerToEigenMatrix<float>(
                tfListener,
                tfMapFrame,
                sensorFrame,
                time_current
            ), mapPointCloud->getHomogeneousDim());
    DP localMap =
        transformation->compute(*newMapPointCloud, T_scanner_to_map.inverse());

    // Cut points in a radius of the parameter sensorMaxRange
    radiusFilter->inPlaceFilter(localMap);

    icpMapLock.lock();
    // Update the transformation to the local map
    this->T_localMap_to_map = T_scanner_to_map;

    icp.setMap(localMap);
    icpMapLock.unlock();
  }
  catch (...) {
    // everything else
    ROS_ERROR_STREAM("Unexpected exception... ignoring scan B");
    return;
  }
}

Mapper::DP *Mapper::updateMap(DP *newPointCloud,
                              const PM::TransformationParameters T_updatedScanner_to_map,
                              bool mapExists) {
  timer t;

  try {
    // Prepare empty field if not existing
    if (newPointCloud->descriptorExists("probabilityStatic") == false) {
      //newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Zero(1, newPointCloud->features.cols()));
      newPointCloud->addDescriptor("probabilityStatic",
                                   PM::Matrix::Constant(1,
                                                        newPointCloud->features.cols(),
                                                        priorStatic));
    }

    if (newPointCloud->descriptorExists("probabilityDynamic") == false) {
      //newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Zero(1, newPointCloud->features.cols()));
      newPointCloud->addDescriptor("probabilityDynamic",
                                   PM::Matrix::Constant(1,
                                                        newPointCloud->features.cols(),
                                                        priorDyn));
    }

    if (newPointCloud->descriptorExists("debug") == false) {
      newPointCloud->addDescriptor("debug",
                                   PM::Matrix::Zero(1,
                                                    newPointCloud->features.cols()));
    }

    if (!mapExists) {
      ROS_DEBUG_STREAM("[MAP] Initial map, only filtering points");
      *newPointCloud =
          transformation->compute(*newPointCloud, T_updatedScanner_to_map);
      mapPostFilters.apply(*newPointCloud);

      return newPointCloud;
    }


    // Early out if no map modification is wanted
    if (!mapping) {
      ROS_DEBUG_STREAM("[MAP] Skipping modification of the map");
      return mapPointCloud;
    }

    const int dimp1(newPointCloud->features.rows());
    const int mapPtsCount(mapPointCloud->getNbPoints());
    const int readPtsCount(newPointCloud->getNbPoints());
    const int dim = mapPointCloud->getEuclideanDim();

    // Build a range image of the reading point cloud (local coordinates)
    PM::Matrix radius_reading =
        newPointCloud->features.topRows(dimp1 - 1).colwise().norm();

    PM::Matrix angles_reading(2, readPtsCount); // 0=inclination, 1=azimuth

    // No atan in Eigen, so we are for to loop through it...
    for (int i = 0; i < readPtsCount; i++) {
      angles_reading(0, i) = 0;
      if (dimp1 == 4) { // 3D only
        const float
            ratio = newPointCloud->features(2, i) / radius_reading(0, i);
        angles_reading(0, i) = acos(ratio);
      }
      angles_reading(1, i) =
          atan2(newPointCloud->features(1, i), newPointCloud->features(0, i));
    }

    std::shared_ptr<NNS> featureNNS;
    featureNNS.reset(NNS::create(angles_reading));


    // Transform the global map in local coordinates
    DP mapLocalFrameCut = transformation->compute(*mapPointCloud,
                                                  T_updatedScanner_to_map.inverse());


    // Remove points out of sensor range
    PM::Matrix globalId(1, mapPtsCount);

    int mapCutPtsCount = 0;
    for (int i = 0; i < mapPtsCount; i++) {
      if (mapLocalFrameCut.features.col(i).head(dimp1 - 1).norm()
          < sensorMaxRange) {
        mapLocalFrameCut.setColFrom(mapCutPtsCount, mapLocalFrameCut, i);
        globalId(0, mapCutPtsCount) = i;
        mapCutPtsCount++;
      }
    }

    mapLocalFrameCut.conservativeResize(mapCutPtsCount);

    PM::Matrix radius_map =
        mapLocalFrameCut.features.topRows(dimp1 - 1).colwise().norm();

    PM::Matrix angles_map(2, mapCutPtsCount); // 0=inclination, 1=azimuth

    // No atan in Eigen, so we need to loop through it...
    for (int i = 0; i < mapCutPtsCount; i++) {
      angles_map(0, i) = 0;
      if (dimp1 == 4) { // 3D
        const float ratio = mapLocalFrameCut.features(2, i) / radius_map(0, i);
        angles_map(0, i) = acos(ratio);
      }

      angles_map(1, i) = atan2(mapLocalFrameCut.features(1, i),
                               mapLocalFrameCut.features(0, i));
    }

    // Look for NN in spherical coordinates
    Matches::Dists dists(1, mapCutPtsCount);
    Matches::Ids ids(1, mapCutPtsCount);

    featureNNS->knn(angles_map,
                    ids,
                    dists,
                    1,
                    0,
                    NNS::ALLOW_SELF_MATCH,
                    maxAngle);

    // Define views on descriptors
    DP::View viewOn_Msec_overlap =
        newPointCloud->getDescriptorViewByName("stamps_Msec");
    DP::View viewOn_sec_overlap =
        newPointCloud->getDescriptorViewByName("stamps_sec");
    DP::View viewOn_nsec_overlap =
        newPointCloud->getDescriptorViewByName("stamps_nsec");

    DP::View viewOnProbabilityStatic =
        mapPointCloud->getDescriptorViewByName("probabilityStatic");
    DP::View viewOnProbabilityDynamic =
        mapPointCloud->getDescriptorViewByName("probabilityDynamic");
    DP::View viewDebug = mapPointCloud->getDescriptorViewByName("debug");

    DP::View viewOn_normals_map =
        mapLocalFrameCut.getDescriptorViewByName("normals");
    DP::View
        viewOn_Msec_map = mapPointCloud->getDescriptorViewByName("stamps_Msec");
    DP::View
        viewOn_sec_map = mapPointCloud->getDescriptorViewByName("stamps_sec");
    DP::View
        viewOn_nsec_map = mapPointCloud->getDescriptorViewByName("stamps_nsec");

    viewDebug = PM::Matrix::Zero(1, mapPtsCount);
    for (int i = 0; i < mapCutPtsCount; i++) {
      if (dists(i) != numeric_limits<float>::infinity()) {
        const int readId = ids(0, i);
        const int mapId = globalId(0, i);

        // in local coordinates
        const Eigen::VectorXf
            readPt = newPointCloud->features.col(readId).head(dimp1 - 1);
        const Eigen::VectorXf
            mapPt = mapLocalFrameCut.features.col(i).head(dimp1 - 1);
        const Eigen::VectorXf mapPt_n = mapPt.normalized();
        const float delta = (readPt - mapPt).norm();
        const float d_max = eps_a * readPt.norm();

        const Eigen::VectorXf normal_map = viewOn_normals_map.col(i);

        // Weight for dynamic elements
        const float w_v = eps + (1. - eps) * fabs(normal_map.dot(mapPt_n));
        const float w_d1 = eps + (1. - eps) * (1. - sqrt(dists(i)) / maxAngle);

        const float offset = delta - eps_d;
        float w_d2 = 1.;
        if (delta < eps_d || mapPt.norm() > readPt.norm()) {
          w_d2 = eps;
        } else {
          if (offset < d_max) {
            w_d2 = eps + (1 - eps) * offset / d_max;
          }
        }

        float w_p2 = eps;
        if (delta < eps_d) {
          w_p2 = 1;
        } else {
          if (offset < d_max) {
            w_p2 = eps + (1. - eps) * (1. - offset / d_max);
          }
        }


        // We don't update point behind the reading
        if ((readPt.norm() + eps_d + d_max) >= mapPt.norm()) {
          const float lastDyn = viewOnProbabilityDynamic(0, mapId);
          const float lastStatic = viewOnProbabilityStatic(0, mapId);

          const float c1 = (1 - (w_v * w_d1));
          const float c2 = w_v * w_d1;


          //Lock dynamic point to stay dynamic under a threshold
          if (lastDyn < maxDyn) {
            viewOnProbabilityDynamic(0, mapId) = c1 * lastDyn
                + c2 * w_d2 * ((1 - alpha) * lastStatic + beta * lastDyn);
            viewOnProbabilityStatic(0, mapId) = c1 * lastStatic
                + c2 * w_p2 * (alpha * lastStatic + (1 - beta) * lastDyn);
          } else {
            viewOnProbabilityStatic(0, mapId) = eps;
            viewOnProbabilityDynamic(0, mapId) = 1 - eps;
          }



          // normalization
          const float sumZ = viewOnProbabilityDynamic(0, mapId)
              + viewOnProbabilityStatic(0, mapId);
          assert(sumZ >= eps);

          viewOnProbabilityDynamic(0, mapId) /= sumZ;
          viewOnProbabilityStatic(0, mapId) /= sumZ;

          viewDebug(0, mapId) = w_d2;


          //TODO use the new time structure
          // Refresh time
          viewOn_Msec_map(0, mapId) = viewOn_Msec_overlap(0, readId);
          viewOn_sec_map(0, mapId) = viewOn_sec_overlap(0, readId);
          viewOn_nsec_map(0, mapId) = viewOn_nsec_overlap(0, readId);
        }

      }
    }

    // Generate temporary map for density computation
    DP tmp_map = mapLocalFrameCut;
    tmp_map.concatenate(*newPointCloud);

    // build and populate NNS
    featureNNS.reset(NNS::create(tmp_map.features,
                                 tmp_map.features.rows() - 1,
                                 NNS::KDTREE_LINEAR_HEAP,
                                 NNS::TOUCH_STATISTICS));

    PM::Matches matches_overlap(
        Matches::Dists(1, readPtsCount),
        Matches::Ids(1, readPtsCount)
    );

    featureNNS->knn(newPointCloud->features,
                    matches_overlap.ids,
                    matches_overlap.dists,
                    1,
                    0);

    DP overlap(newPointCloud->createSimilarEmpty());
    DP no_overlap(newPointCloud->createSimilarEmpty());

    int ptsOut = 0;
    int ptsIn = 0;
    for (int i = 0; i < readPtsCount; ++i) {
      if (matches_overlap.dists(i) > maxDistNewPoint) {
        no_overlap.setColFrom(ptsOut, *newPointCloud, i);
        ptsOut++;
      } else {
        overlap.setColFrom(ptsIn, *newPointCloud, i);
        ptsIn++;
      }
    }

    no_overlap.conservativeResize(ptsOut);
    overlap.conservativeResize(ptsIn);

    // Initialize descriptors
    no_overlap.addDescriptor("probabilityStatic",
                             PM::Matrix::Constant(1,
                                                  no_overlap.features.cols(),
                                                  priorStatic));
    no_overlap.addDescriptor("probabilityDynamic",
                             PM::Matrix::Constant(1,
                                                  no_overlap.features.cols(),
                                                  priorDyn));
    no_overlap.addDescriptor("debug",
                             PM::Matrix::Zero(1, no_overlap.features.cols()));

    // shrink the newPointCloud to the new information
    *newPointCloud = no_overlap;

    // Correct new points using ICP result
    *newPointCloud =
        transformation->compute(*newPointCloud, T_updatedScanner_to_map);

    // Merge point clouds to map
    newPointCloud->concatenate(*mapPointCloud);
    mapPostFilters.apply(*newPointCloud);
  }
  catch (DP::InvalidField e) {
    ROS_ERROR_STREAM(e.what());
    abort();
  }
  catch (const std::exception &exc) {
    // catch anything thrown within try block that derives from std::exception
    std::cerr << exc.what();
    abort();
  }

  ROS_DEBUG_STREAM(
      "[TIME][MAP] New map available (" << newPointCloud->features.cols()
                                        << " pts), update took " << t.elapsed()
                                        << " [s]");

  return newPointCloud;
}

void Mapper::waitForMapBuildingCompleted() {
#if BOOST_VERSION >= 104100
  if (mapBuildingInProgress)
  {
      // we wait for now, in future we should kill it
      mapBuildingFuture.wait();
      mapBuildingInProgress = false;
  }
#endif // BOOST_VERSION >= 104100
}

void Mapper::publishLoop(double publishPeriod) {
  if (publishPeriod == 0)
    return;
  ros::Rate r(1.0 / publishPeriod);
  while (ros::ok()) {
    publishTransform();
    r.sleep();
  }
}

void Mapper::publishTransform() {
}

bool Mapper::getPointMap(map_msgs::GetPointMap::Request &req,
                         map_msgs::GetPointMap::Response &res) {
  if (!mapPointCloud)
    return false;

  // FIXME: do we need a mutex here?
  res.map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud,
                                                               mapFrame,
                                                               ros::Time::now());
  return true;
}

bool Mapper::saveMap(map_msgs::SaveMap::Request &req,
                     map_msgs::SaveMap::Response &res) {
  if (!mapPointCloud)
    return false;

  try {
    mapPointCloud->save(req.filename.data);
  }
  catch (const std::runtime_error &e) {
    ROS_ERROR_STREAM("Unable to save: " << e.what());
    return false;
  }

  ROS_INFO_STREAM("[MAP] saved at " << req.filename.data << " with "
                                    << mapPointCloud->features.cols()
                                    << " points.");
  return true;
}

bool Mapper::loadMap(ethzasl_icp_mapper::LoadMap::Request &req,
                     ethzasl_icp_mapper::LoadMap::Response &res) {
  waitForMapBuildingCompleted();

  DP *cloud(new DP(DP::load(req.filename.data)));

  // Print new map information
  const int dim = cloud->features.rows();
  const int nbPts = cloud->features.cols();
  ROS_INFO_STREAM(
      "[MAP] Loading " << dim - 1 << "D point cloud (" << req.filename.data
                       << ") with " << nbPts << " points.");

  ROS_INFO_STREAM("  With descriptors:");
  for (int i = 0; i < cloud->descriptorLabels.size(); i++) {
    ROS_INFO_STREAM("    - " << cloud->descriptorLabels[i].text);
  }

  T_localMap_to_map = PM::TransformationParameters::Identity(dim, dim);
  T_scanner_to_map = PM::TransformationParameters::Identity(dim, dim);
  //TODO: check that...
  mapping = true;
  setMap(updateMap(cloud,
                   PM::TransformationParameters::Identity(dim, dim),
                   false));
  mapping = false;

  return true;
}

bool Mapper::reset(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res) {
  waitForMapBuildingCompleted();

  // note: no need for locking as we do ros::spin(), to update if we go for multi-threading
  publishLock.lock();
  // WARNING: this will break in 2D
  T_localMap_to_map = PM::TransformationParameters::Identity(4, 4);
  T_scanner_to_map = PM::TransformationParameters::Identity(4, 4);
  publishLock.unlock();

  icp.clearMap();

  return true;
}

bool Mapper::correctPose(ethzasl_icp_mapper::CorrectPose::Request &req,
                         ethzasl_icp_mapper::CorrectPose::Response &res) {
  publishLock.lock();
  const int dim = mapPointCloud->getHomogeneousDim();

  try {

    T_scanner_to_map =
        PointMatcher_ros::eigenMatrixToDim<float>(
            PointMatcher_ros::transformListenerToEigenMatrix<float>(
                tfListener,
                tfMapFrame,
                sensorFrame,
                ros::Time::now()
            ), dim);


    // update ICP map
    updateIcpMap(mapPointCloud);

  }
  catch (...) {
    // everything else
    publishLock.unlock();
    ROS_ERROR_STREAM("Unexpected exception... ignoring scan C");
    return false;
  }

  publishLock.unlock();

  return true;
}

bool Mapper::setMode(ethzasl_icp_mapper::SetMode::Request &req,
                     ethzasl_icp_mapper::SetMode::Response &res) {
  // Impossible states
  if (req.localize == false && req.map == true)
    return false;

  localizing = req.localize;
  mapping = req.map;

  return true;
}

bool Mapper::getMode(ethzasl_icp_mapper::GetMode::Request &req,
                     ethzasl_icp_mapper::GetMode::Response &res) {
  res.localize = localizing;
  res.map = mapping;
  return true;
}

bool Mapper::initialTransform(ethzasl_icp_mapper::InitialTransform::Request &req,
                              ethzasl_icp_mapper::InitialTransform::Response &res) {
  posePub.publish(req.transform);
  return true;
}

bool Mapper::loadPublishedMap(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res) {
  cad_trigger = true;
}

bool Mapper::getBoundedMap(ethzasl_icp_mapper::GetBoundedMap::Request &req,
                           ethzasl_icp_mapper::GetBoundedMap::Response &res) {
  if (!mapPointCloud)
    return false;

  const float max_x = req.topRightCorner.x;
  const float max_y = req.topRightCorner.y;
  const float max_z = req.topRightCorner.z;

  const float min_x = req.bottomLeftCorner.x;
  const float min_y = req.bottomLeftCorner.y;
  const float min_z = req.bottomLeftCorner.z;

  tf::StampedTransform stampedTr;

  Eigen::Affine3d eigenTr;
  tf::poseMsgToEigen(req.mapCenter, eigenTr);
  Eigen::MatrixXf T = eigenTr.matrix().inverse().cast<float>();

  T = transformation->correctParameters(T);


  // FIXME: do we need a mutex here?
  const DP centeredPointCloud = transformation->compute(*mapPointCloud, T);
  DP cutPointCloud = centeredPointCloud.createSimilarEmpty();

  int newPtCount = 0;
  for (int i = 0; i < centeredPointCloud.features.cols(); i++) {
    const float x = centeredPointCloud.features(0, i);
    const float y = centeredPointCloud.features(1, i);
    const float z = centeredPointCloud.features(2, i);

    if (x < max_x && x > min_x &&
        y < max_y && y > min_y &&
        z < max_z && z > min_z) {
      cutPointCloud.setColFrom(newPtCount, centeredPointCloud, i);
      newPtCount++;
    }
  }

  ROS_INFO_STREAM("Extract " << newPtCount << " points from the map");

  cutPointCloud.conservativeResize(newPtCount);
  cutPointCloud = transformation->compute(cutPointCloud, T.inverse());


  // Send the resulting point cloud in ROS format
  res.boundedMap = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
      cutPointCloud,
      mapFrame,
      ros::Time::now());
  return true;
}

void Mapper::loadExternalParameters() {

  // load configs
  string configFileName;
  if (ros::param::get("~icpConfig", configFileName)) {
    ifstream ifs(configFileName.c_str());
    if (ifs.good()) {
      icp.loadFromYaml(ifs);
    } else {
      ROS_ERROR_STREAM(
          "Cannot load ICP config from YAML file " << configFileName);
      icp.setDefault();
    }
  } else {
    ROS_INFO_STREAM("No ICP config file given, using default");
    icp.setDefault();
  }

  if (ros::param::get("~inputFiltersConfig", configFileName)) {
    ifstream ifs(configFileName.c_str());
    if (ifs.good()) {
      inputFilters = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM(
          "Cannot load input filters config from YAML file " << configFileName);
    }
  } else {
    ROS_INFO_STREAM(
        "No input filters config file given, not using these filters");
  }

  if (ros::param::get("~mapPreFiltersConfig", configFileName)) {
    ifstream ifs(configFileName.c_str());
    if (ifs.good()) {
      mapPreFilters = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM("Cannot load map pre-filters config from YAML file "
                           << configFileName);
    }
  } else {
    ROS_INFO_STREAM(
        "No map pre-filters config file given, not using these filters");
  }

  if (ros::param::get("~mapPostFiltersConfig", configFileName)) {
    ifstream ifs(configFileName.c_str());
    if (ifs.good()) {
      mapPostFilters = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file "
                           << configFileName);
    }
  } else {
    ROS_INFO_STREAM(
        "No map post-filters config file given, not using these filters");
  }
}

bool Mapper::reloadallYaml(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &res) {
  loadExternalParameters();
  ROS_INFO_STREAM("Parameters reloaded");

  return true;
}

}

//// Main function supporting the Mapper class
//int main(int argc, char **argv) {
//  ros::init(argc, argv, "mapper");
//  ros::NodeHandle n;
//  ros::NodeHandle pn("~");
//  Mapper mapper(n, pn);
//  ros::spin();
//
//  return 0;
//}
