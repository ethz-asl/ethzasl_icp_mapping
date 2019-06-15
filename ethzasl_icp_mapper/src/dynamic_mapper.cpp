#include "nav_msgs/Odometry.h"
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
    T_localMap_to_map(PM::TransformationParameters::Identity(4, 4)),
    T_scanner_to_map(PM::TransformationParameters::Identity(4, 4)),
    publishStamp(ros::Time::now()),
    tfListener(ros::Duration(30)),
    odom_received(0) {

  // set logger
  if (parameters_.use_logger) {
    PointMatcherSupport::setLogger(make_shared<PointMatcherSupport::ROSLogger>());
  }

  // Load all parameters stored in external files
  loadExternalParameters();

  PM::Parameters params;
  params["dim"] = "-1";
  params["maxDist"] = toParam(parameters_.sensor_max_range);

  radiusFilter = PM::get().DataPointsFilterRegistrar.create(
      "MaxDistDataPointsFilter",
      params);

  // topic initializations
  if (parameters_.subscribe_cloud) {
    cloudSub = n.subscribe("cloud_in",
                           parameters_.input_queue_size,
                           &Mapper::gotCloud,
                           this);
  }
  if (parameters_.subscribe_cad) {
    cadSub = n.subscribe("cad_interface_node/cad_model",
                         parameters_.input_queue_size,
                         &Mapper::gotCAD,
                         this);
  }

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
}

Mapper::~Mapper() {
#if BOOST_VERSION >= 104100
  // wait for map-building thread
  if (mapBuildingInProgress) {
    mapBuildingFuture.wait();
    if (mapBuildingFuture.has_value())
      delete mapBuildingFuture.get();
  }
#endif // BOOST_VERSION >= 104100
  // wait for publish thread
  publishThread.join();
  // save point cloud
  if (mapPointCloud) {
    mapPointCloud->save(parameters_.vtk_final_map_name);
    delete mapPointCloud;
  }
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2 &cloudMsgIn) {
  if (parameters_.localizing) {
    if (odom_received < 3) {
      try {
        tf::StampedTransform transform;
        tfListener.lookupTransform(parameters_.tf_map_frame,
                                   parameters_.lidar_frame,
                                   cloudMsgIn.header.stamp,
                                   transform);
        odom_received++;
      } catch (tf::TransformException ex) {
        ROS_WARN_STREAM("Transformations still initializing.");
        posePub.publish(PointMatcher_ros::eigenMatrixToTransformStamped<float>(
            T_scanner_to_map.inverse(),
            parameters_.lidar_frame,
            parameters_.tf_map_frame,
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
  if (parameters_.cad_trigger) {
    ROS_WARN_STREAM("Processing CAD");
    std::cout << "getting cad" << std::endl;
    // Load the cad map as base map.
    parameters_.localizing = true;
    parameters_.mapping = true;
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    reset(req, res);
    unique_ptr<DP> cloud
        (new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
    processCloud(move(cloud),
                 cloudMsgIn.header.frame_id,
                 cloudMsgIn.header.stamp,
                 cloudMsgIn.header.seq);
    parameters_.mapping = false;
    parameters_.cad_trigger = false;
    publishLock.unlock();
  }
}

void Mapper::processCloud(unique_ptr<DP> newPointCloud,
                          const std::string &scannerFrame,
                          const ros::Time &stamp,
                          uint32_t seq) {

  // If the sensor frame was not set by the user, use default
  if (parameters_.sensor_frame== "") {
    parameters_.sensor_frame= scannerFrame;
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
  }

  // Ensure a minimum amount of point before filtering
  int ptsCount = newPointCloud->getNbPoints();
  if (ptsCount < parameters_.min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << ptsCount
                                                          << " pts.");
    return;
  }

  {

    // Apply filters to incoming cloud, in scanner coordinates
    inputFilters.apply(*newPointCloud);
  }

  try {
    T_scanner_to_map = PointMatcher_ros::eigenMatrixToDim<float>(
        PointMatcher_ros::transformListenerToEigenMatrix<float>(
            tfListener,
            parameters_.tf_map_frame, // to
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
    ROS_ERROR_STREAM("Unexpected exception... ignoring scan.");
    return;
  }
  ROS_DEBUG_STREAM(
      "[ICP] T_scanner_to_map (" << scannerFrame << " to "
                                 << parameters_.map_frame<< "):\n"
                                 << T_scanner_to_map);

  const PM::TransformationParameters T_scanner_to_localMap =
      transformation->correctParameters(
          T_localMap_to_map.inverse() * T_scanner_to_map);

  // Ensure a minimum amount of point after filtering
  ptsCount = newPointCloud->getNbPoints();
  if (ptsCount < parameters_.min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << ptsCount
                                                          << " pts.");
    return;
  }

  // Initialize the map if empty
  if (!icp.hasMap()) {
    ROS_INFO_STREAM("[MAP] Creating an initial map");
    mapCreationTime = stamp;
    setMap(updateMap(newPointCloud.release(), T_scanner_to_map, false));
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
                                      << icp.getPrefilteredInternalMap().getNbPoints());

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
    if (estimatedOverlap < parameters_.min_overlap) {
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
          parameters_.tf_map_frame,
          stamp));
    }
    // Publish pose
    if (posePub.getNumSubscribers()) {
      // Not sure that the transformation represents the odometry
      posePub.publish(PointMatcher_ros::eigenMatrixToTransformStamped<float>(
          T_updatedScanner_to_map,
          parameters_.lidar_frame,
          parameters_.tf_map_frame,
          stamp));
    }
    if (mapPub.getNumSubscribers()) {
      mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud,
                                                                        parameters_.tf_map_frame,
                                                                        mapCreationTime));
    }
    // Publish the corrected scan point cloud
    DP pc = transformation->compute(*newPointCloud, T_updatedScanner_to_map);
    mapPostFilters.apply(pc);
    publishLock.lock();
    if (scanPub.getNumSubscribers() && parameters_.localizing) {
      ROS_DEBUG_STREAM(
          "Corrected scan publishing " << pc.getNbPoints() << " points");
      scanPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pc,
                                                                         parameters_.tf_map_frame,
                                                                         stamp));
    }
    publishLock.unlock();

    // check if new points should be added to the map.
    if (
        ((estimatedOverlap < parameters_.max_overlap_to_merge)
            || (icp.getPrefilteredInternalMap().features.cols()
                < parameters_.min_map_point_count)) && (!mapBuildingInProgress)
        ) {
      // Make sure we process the last available map.
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
           << (estimatedOverlap < parameters_.max_overlap_to_merge) << endl;
      cerr
          << "(icp.getPrefilteredInternalMap().features.cols() < minMapPointCount): "
          << icp.getPrefilteredInternalMap().features.cols() << " < "
          << parameters_.min_map_point_count
          << " = " << (icp.getPrefilteredInternalMap().features.cols()
          < parameters_.min_map_point_count)
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

  ROS_INFO_STREAM("[TIME] Total ICP took: " << t.elapsed() << " [s]");
  if (realTimeRatio < 80)
    ROS_INFO_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
  else
    ROS_WARN_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

  lastPoinCloudTime = stamp;
  lastPointCloudSeq = seq;
}

void Mapper::processNewMapIfAvailable() {
#if BOOST_VERSION >= 104100
  if (mapBuildingInProgress && mapBuildingFuture.has_value()) {
    ROS_DEBUG_STREAM("[MAP] Computation in thread done");
    setMap(mapBuildingFuture.get());
    mapBuildingInProgress = false;
  }
#endif // BOOST_VERSION >= 104100
}

void Mapper::setMap(DP *newMapPointCloud) {

  // Delete old map.
  if (mapPointCloud && mapPointCloud != newMapPointCloud)
    delete mapPointCloud;

  // Set new map.
  mapPointCloud = newMapPointCloud;

  // Update ICP map.
  updateIcpMap(mapPointCloud);

  // Publish map point cloud.
  publishLock.lock();
  if (mapPub.getNumSubscribers() && parameters_.mapping) {
    ROS_DEBUG_STREAM(
        "[MAP] publishing " << mapPointCloud->getNbPoints() << " points");
    mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud,
                                                                      parameters_.tf_map_frame,
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
                parameters_.tf_map_frame,
                parameters_.sensor_frame,
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
      newPointCloud->addDescriptor("probabilityStatic",
                                   PM::Matrix::Constant(1,
                                                        newPointCloud->features.cols(),
                                                        parameters_.prior_static));
    }

    if (newPointCloud->descriptorExists("probabilityDynamic") == false) {
      newPointCloud->addDescriptor("probabilityDynamic",
                                   PM::Matrix::Constant(1,
                                                        newPointCloud->features.cols(),
                                                        parameters_.prior_dyn));
    }

    if (!mapExists) {
      ROS_DEBUG_STREAM("[MAP] Initial map, only filtering points");
      *newPointCloud =
          transformation->compute(*newPointCloud, T_updatedScanner_to_map);
      mapPostFilters.apply(*newPointCloud);

      return newPointCloud;
    }


    // Early out if no map modification is wanted
    if (!parameters_.mapping) {
      ROS_DEBUG_STREAM("[MAP] Skipping modification of the map");
      return mapPointCloud;
    }

    const int dimp1(newPointCloud->features.rows());
    const int mapPtsCount(mapPointCloud->getNbPoints());
    const int readPtsCount(newPointCloud->getNbPoints());
    const int dim = mapPointCloud->getEuclideanDim();

    std::shared_ptr<NNS> featureNNS;
    // Transform the global map in local coordinates
    DP mapLocalFrameCut = transformation->compute(*mapPointCloud,
                                                  T_updatedScanner_to_map.inverse());
    // Perform density computation.
    mapLocalFrameCut.concatenate(*newPointCloud);

    // build and populate NNS
    featureNNS.reset(NNS::create(mapLocalFrameCut.features,
                                 mapLocalFrameCut.features.rows() - 1,
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

    DP no_overlap(newPointCloud->createSimilarEmpty());

    int ptsOut = 0;
    for (int i = 0; i < readPtsCount; ++i) {
      if (matches_overlap.dists(i) > parameters_.max_dist_new_point) {
        no_overlap.setColFrom(ptsOut, *newPointCloud, i);
        ptsOut++;
      }
    }

    no_overlap.conservativeResize(ptsOut);

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

  ROS_INFO_STREAM(
      "[TIME][MAP] New map available (" << newPointCloud->features.cols()
                                        << " pts), update took " << t.elapsed()
                                        << " [s]");

  return newPointCloud;
}

void Mapper::waitForMapBuildingCompleted() {
#if BOOST_VERSION >= 104100
  if (mapBuildingInProgress) {
    // we wait for now, in future we should kill it
    mapBuildingFuture.wait();
    mapBuildingInProgress = false;
  }
#endif // BOOST_VERSION >= 104100
}

bool Mapper::getPointMap(map_msgs::GetPointMap::Request &req,
                         map_msgs::GetPointMap::Response &res) {
  if (!mapPointCloud)
    return false;

  // FIXME: do we need a mutex here?
  res.map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud,
                                                               parameters_.map_frame,
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
  parameters_.mapping = true;
  setMap(updateMap(cloud,
                   PM::TransformationParameters::Identity(dim, dim),
                   false));
  parameters_.mapping = false;

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
                parameters_.tf_map_frame,
                parameters_.sensor_frame,
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

  parameters_.localizing = req.localize;
  parameters_.mapping = req.map;

  return true;
}

bool Mapper::getMode(ethzasl_icp_mapper::GetMode::Request &req,
                     ethzasl_icp_mapper::GetMode::Response &res) {
  res.localize = parameters_.localizing;
  res.map = parameters_.mapping;
  return true;
}

bool Mapper::initialTransform(ethzasl_icp_mapper::InitialTransform::Request &req,
                              ethzasl_icp_mapper::InitialTransform::Response &res) {
  posePub.publish(req.transform);
  return true;
}

bool Mapper::loadPublishedMap(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res) {
  parameters_.cad_trigger = true;
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
      parameters_.map_frame,
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