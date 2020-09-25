#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include "ethzasl_icp_mapper/dynamic_mapper.h"

namespace mapper {

Mapper::Mapper(ros::NodeHandle &n, ros::NodeHandle &pn) :
    n(n),
    pn(pn),
    map_point_cloud_(0),
    transformation_(PM::get().REG(Transformation).create
        ("RigidTransformation")),
#if BOOST_VERSION >= 104100
    map_building_in_progress_(false),
#endif // BOOST_VERSION >= 104100
    T_local_map_to_map_(PM::TransformationParameters::Identity(4, 4)),
    T_scanner_to_map_(PM::TransformationParameters::Identity(4, 4)),
    tf_listener_(ros::Duration(30)),
    odom_received_(0),
    scan_counter_(0) {

  if (parameters_.use_logger) {
    PointMatcherSupport::setLogger(make_shared<PointMatcherSupport::ROSLogger>());
  }

  loadExternalParameters();
  PM::Parameters params;
  params["dim"] = "-1";
  params["maxDist"] = toParam(parameters_.sensor_max_range);

  radius_filter_ = PM::get().DataPointsFilterRegistrar.create(
      "MaxDistDataPointsFilter",
      params);

  if (parameters_.subscribe_cloud) {
    cloud_sub_ = n.subscribe("cloud_in",
                             parameters_.input_queue_size,
                             &Mapper::gotCloud,
                             this);
  }
  if (parameters_.subscribe_map) {
    map_sub_ = n.subscribe("map_interface_node/map_model",
                           parameters_.input_queue_size,
                           &Mapper::gotMap,
                           this);
  }

  map_pub_ = n.advertise<sensor_msgs::PointCloud2>("point_map", 2, true);
  scan_pub_ = n.advertise<sensor_msgs::PointCloud2>("corrected_scan", 2, true);
  outlier_pub_ = n.advertise<sensor_msgs::PointCloud2>("outliers", 2, true);
  odom_pub_ = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
  pose_pub_ =
      n.advertise<geometry_msgs::TransformStamped>("icp_pose", 50, true);
  odom_error_pub_ = n.advertise<nav_msgs::Odometry>("icp_error_odom", 50, true);

  get_point_map_srv_ =
      n.advertiseService("dynamic_point_map", &Mapper::getPointMap, this);
  save_map_srv_ = pn.advertiseService("save_map", &Mapper::saveMap, this);
  load_map_srv_ = pn.advertiseService("load_map", &Mapper::loadMap, this);
  reset_srv_ = pn.advertiseService("reset", &Mapper::reset, this);
  correct_pose_srv_ =
      pn.advertiseService("correct_pose", &Mapper::correctPose, this);
  initial_transform_srv_ =
      pn.advertiseService("intial_transform", &Mapper::initialTransform, this);
  load_published_map_srv_ = pn.advertiseService("load_published_map",
                                                &Mapper::loadPublishedMap,
                                                this);
  set_mode_srv_ = pn.advertiseService("set_mode", &Mapper::setMode, this);
  get_mode_srv_ = pn.advertiseService("get_mode", &Mapper::getMode, this);
  get_bounded_map_srv_ =
      pn.advertiseService("get_bounded_map", &Mapper::getBoundedMap, this);
  reload_all_yaml_srv_ =
      pn.advertiseService("reload_all_yaml", &Mapper::reloadallYaml, this);
}

Mapper::~Mapper() {
#if BOOST_VERSION >= 104100
  // Wait for map-building thread.
  if (map_building_in_progress_) {
    map_building_future_.wait();
    if (map_building_future_.has_value())
      delete map_building_future_.get();
  }
#endif // BOOST_VERSION >= 104100
  // Wait for publish thread.
  publish_thread_.join();
  // Save point cloud.
  if (map_point_cloud_) {
    map_point_cloud_->save(parameters_.vtk_final_map_name);
    std::string pcd_name = "finalMap.pcd";
    PointMatcherIO<float>::savePCD(*map_point_cloud_, pcd_name);
    delete map_point_cloud_;
  }
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  if (parameters_.localizing) {
    if (odom_received_ < 3) {
      try {
        tf::StampedTransform transform;
        tf_listener_.lookupTransform(parameters_.tf_map_frame,
                                     parameters_.lidar_frame,
                                     cloud_msg_in.header.stamp,
                                     transform);
        odom_received_++;
      } catch (tf::TransformException ex) {
        ROS_WARN_STREAM("Transformations still initializing.");
        pose_pub_.publish(PointMatcher_ros::eigenMatrixToTransformStamped
                              <float>(
            T_scanner_to_map_.inverse(),
            parameters_.lidar_frame,
            parameters_.tf_map_frame,
            cloud_msg_in.header.stamp));
        odom_received_++;
      }
    } else {
        if (scan_counter_ == parameters_.skip_frames) {
        unique_ptr<DP> cloud(new DP(
            PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in)));

        processCloud(move(cloud), parameters_.lidar_frame,
                     cloud_msg_in.header.stamp, cloud_msg_in.header.seq, false);
            scan_counter_ = 0;
      } else {
            ++scan_counter_;
            ROS_INFO_STREAM("Skipping frame " << scan_counter_ << "/" << parameters_.skip_frames);
      }
    }
  }
}

void Mapper::gotMap(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  if (parameters_.map_trigger) {
    ROS_WARN_STREAM("Processing loaded map");
    // Load the map as base map.
    parameters_.localizing = true;
    parameters_.mapping = true;
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    reset(req, res);
    unique_ptr<DP> cloud
        (new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in)));
    processCloud(move(cloud),
                 cloud_msg_in.header.frame_id,
                 cloud_msg_in.header.stamp,
                 cloud_msg_in.header.seq, true);
    parameters_.map_trigger = false;
    publish_lock_.unlock();
  }
}

void Mapper::processCloud(unique_ptr<DP> new_point_cloud,
                          const std::string &scanner_frame,
                          const ros::Time &stamp,
                          uint32_t seq, bool new_map) {

  if (parameters_.sensor_frame == "") {
    parameters_.sensor_frame = scanner_frame;
  }

  // If the future has completed, use the new map.
  processNewMapIfAvailable();

  timer t;

  const size_t good_count(new_point_cloud->features.cols());
  if (good_count == 0) {
    ROS_ERROR("[ICP] I found no good points in the cloud");
    return;
  }

  // Dimension of the point cloud, important since we handle 2D and 3D.
  const int dimp1(new_point_cloud->features.rows());

  // This need to be depreciated, there is addTime for those field in pm.
  if (!(new_point_cloud->descriptorExists("stamps_Msec")
      && new_point_cloud->descriptorExists("stamps_sec")
      && new_point_cloud->descriptorExists("stamps_nsec"))) {
    const float Msec = round(stamp.sec / 1e6);
    const float sec = round(stamp.sec - Msec * 1e6);
    const float nsec = round(stamp.nsec);

    const PM::Matrix desc_Msec = PM::Matrix::Constant(1, good_count, Msec);
    const PM::Matrix desc_sec = PM::Matrix::Constant(1, good_count, sec);
    const PM::Matrix desc_nsec = PM::Matrix::Constant(1, good_count, nsec);
    new_point_cloud->addDescriptor("stamps_Msec", desc_Msec);
    new_point_cloud->addDescriptor("stamps_sec", desc_sec);
    new_point_cloud->addDescriptor("stamps_nsec", desc_nsec);
  }

  int pts_count = new_point_cloud->getNbPoints();
  if (pts_count < parameters_.min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << pts_count
                                                          << " pts.");
    return;
  }

  input_filters_.apply(*new_point_cloud);

  try {
    T_scanner_to_map_ = PointMatcher_ros::eigenMatrixToDim<float>(
        PointMatcher_ros::transformListenerToEigenMatrix<float>(
            tf_listener_,
            parameters_.tf_map_frame, // to
            scanner_frame, // from
            stamp
        ), dimp1);
  } catch (tf::ExtrapolationException e) {
    ROS_ERROR_STREAM("Extrapolation Exception. stamp = " << stamp << " now = "
                                                         << ros::Time::now()
                                                         << " delta = "
                                                         << ros::Time::now()
                                                             - stamp << endl
                                                         << e.what());
    return;
  } catch (...) {
    // Everything else.
    ROS_ERROR_STREAM("Unexpected exception... ignoring scan.");
    return;
  }
  ROS_DEBUG_STREAM(
      "[ICP] T_scanner_to_map (" << scanner_frame << " to "
                                 << parameters_.map_frame << "):\n"
                                 << T_scanner_to_map_);

  const PM::TransformationParameters T_scanner_to_local_map =
      transformation_->correctParameters(
          T_local_map_to_map_.inverse() * T_scanner_to_map_);

  pts_count = new_point_cloud->getNbPoints();
  if (pts_count < parameters_.min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << pts_count
                                                          << " pts.");
    return;
  }

  if (!icp_.hasMap() || new_map) {
    ROS_INFO_STREAM("[MAP] Creating an initial map");
    map_creation_time_ = stamp;
    setMap(updateMap(new_point_cloud.release(), T_scanner_to_map_, false));
    parameters_.map_trigger = false;
    return;
  }

  if (new_point_cloud->getEuclideanDim()
      != icp_.getPrefilteredInternalMap().getEuclideanDim()) {
    ROS_ERROR_STREAM("[ICP] Dimensionality missmatch: incoming cloud is "
                         << new_point_cloud->getEuclideanDim()
                         << " while map is "
                         << icp_.getPrefilteredInternalMap().getEuclideanDim());
    return;
  }

  try {
    PM::TransformationParameters T_updated_scanner_to_map;
    PM::TransformationParameters T_updated_scanner_to_local_map;

    ROS_DEBUG_STREAM(
        "[ICP] Computing - reading: " << new_point_cloud->getNbPoints()
                                      << ", reference: "
                                      << icp_.getPrefilteredInternalMap()
                                          .getNbPoints());

    icp_map_lock_.lock();
    T_updated_scanner_to_local_map = icp_(*new_point_cloud,
                                          T_scanner_to_local_map);
    icp_map_lock_.unlock();

    T_updated_scanner_to_map = T_local_map_to_map_ *
        T_updated_scanner_to_local_map;

    ROS_DEBUG_STREAM(
        "[ICP] T_updatedScanner_to_map:\n" << T_updated_scanner_to_map);
    ROS_DEBUG_STREAM("[ICP] T_updatedScanner_to_localMap:\n"
                         << T_updated_scanner_to_local_map);

    // Ensure minimum overlap between scans.
    const double estimated_overlap = icp_.errorMinimizer->getOverlap();
    ROS_DEBUG_STREAM("[ICP] Overlap: " << estimated_overlap);
    if (estimated_overlap < parameters_.min_overlap) {
      ROS_ERROR_STREAM(
          "[ICP] Estimated overlap too small, ignoring ICP correction!");
      return;
    }

    // Publish odometry.
    if (odom_pub_.getNumSubscribers()) {
      odom_pub_.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(
          T_updated_scanner_to_map,
          parameters_.tf_map_frame,
          stamp));
    }
    // Publish pose.
    if (pose_pub_.getNumSubscribers()) {
      pose_pub_.publish(PointMatcher_ros::eigenMatrixToTransformStamped<float>(
          T_updated_scanner_to_map,
          parameters_.lidar_frame,
          parameters_.tf_map_frame,
          stamp));
    }
    if (map_pub_.getNumSubscribers()) {
      map_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
                           (*map_point_cloud_,
                            parameters_.tf_map_frame,
                            map_creation_time_));
    }
    // Publish the corrected scan point cloud
    DP pc = transformation_->compute(*new_point_cloud,
                                     T_updated_scanner_to_map);
    map_post_filters_.apply(pc);
    publish_lock_.lock();
    if (scan_pub_.getNumSubscribers() && parameters_.localizing) {
      ROS_DEBUG_STREAM(
          "Corrected scan publishing " << pc.getNbPoints() << " points");
      scan_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pc,
                                                                           parameters_.tf_map_frame,
                                                                           stamp));
    }
    publish_lock_.unlock();

    if (
        ((estimated_overlap < parameters_.max_overlap_to_merge)
            || (icp_.getPrefilteredInternalMap().features.cols()
                < parameters_.min_map_point_count)) &&
            (!map_building_in_progress_)
        ) {
      // Make sure we process the last available map.
      map_creation_time_ = stamp;

      ROS_DEBUG_STREAM("[MAP] Adding new points in a separate thread");

      map_building_task_ = MapBuildingTask(boost::bind(&Mapper::updateMap,
                                                       this,
                                                       new_point_cloud.release(),
                                                       T_updated_scanner_to_map,
                                                       true));
      map_building_future_ = map_building_task_.get_future();
      map_building_thread_ =
          boost::thread(boost::move(boost::ref(map_building_task_)));
      map_building_thread_.detach(); // We don't care about joining this one.
      sched_yield();
      map_building_in_progress_ = true;
    } else {
      cerr << "SKIPPING MAP" << endl;
      cerr << "estimatedOverlap < maxOverlapToMerge: "
           << (estimated_overlap < parameters_.max_overlap_to_merge) << endl;
      cerr
          << "(icp.getPrefilteredInternalMap().features.cols() < minMapPointCount): "
          << icp_.getPrefilteredInternalMap().features.cols() << " < "
          << parameters_.min_map_point_count
          << " = " << (icp_.getPrefilteredInternalMap().features.cols()
          < parameters_.min_map_point_count)
          << endl;
      cerr << "mapBuildingInProgress: " << map_building_in_progress_ << endl;

      bool state_lock = publish_lock_.try_lock();
      if (state_lock) publish_lock_.unlock();
      cerr << "publishLock.try_lock(): " << state_lock << endl;

      state_lock = icp_map_lock_.try_lock();
      if (state_lock) icp_map_lock_.unlock();
      cerr << "icpMapLock.try_lock(): " << state_lock << endl;

      cerr << "mapBuildingFuture.has_value(): " << map_building_future_
          .has_value()
           << endl;

    }

  } catch (PM::ConvergenceError error) {
    icp_map_lock_.unlock();
    ROS_ERROR_STREAM("[ICP] failed to converge: " << error.what());
    new_point_cloud->save("error_read.vtk");
    icp_.getPrefilteredMap().save("error_ref.vtk");
    return;
  } catch (...) {
    // everything else.
    publish_lock_.unlock();
    ROS_ERROR_STREAM("Unen XZ");
    return;
  }
  // Statistics about time and real-time capability.
  int real_time_ratio =
      100 * t.elapsed() / (stamp.toSec() - last_poin_cloud_time_.toSec());
  real_time_ratio *= seq - last_point_cloud_seq_;

  ROS_INFO_STREAM("[TIME] Total ICP took: " << t.elapsed() << " [s]");
  if (real_time_ratio < 80)
    ROS_INFO_STREAM("[TIME] Real-time capability: " << real_time_ratio <<
                                                    "%");
  else
    ROS_WARN_STREAM("[TIME] Real-time capability: " << real_time_ratio << "%");

  last_poin_cloud_time_ = stamp;
  last_point_cloud_seq_ = seq;
}

void Mapper::processNewMapIfAvailable() {
#if BOOST_VERSION >= 104100
  if (map_building_in_progress_ && map_building_future_.has_value()) {
    ROS_DEBUG_STREAM("[MAP] Computation in thread done");
    setMap(map_building_future_.get());
    map_building_in_progress_ = false;
  }
#endif // BOOST_VERSION >= 104100
}

void Mapper::setMap(DP *newMapPointCloud) {

  // Delete old map.
  if (map_point_cloud_ && map_point_cloud_ != newMapPointCloud)
    delete map_point_cloud_;

  // Set new map.
  map_point_cloud_ = newMapPointCloud;

  // Update ICP map.
  updateIcpMap(map_point_cloud_);

  // Publish map point cloud.
  publish_lock_.lock();
  if (map_pub_.getNumSubscribers() && parameters_.mapping) {
    ROS_DEBUG_STREAM(
        "[MAP] publishing " << map_point_cloud_->getNbPoints() << " points");
    map_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
                         (*map_point_cloud_,
                          parameters_.tf_map_frame,
                          map_creation_time_));
  }
  publish_lock_.unlock();
}

void Mapper::updateIcpMap(const DP *new_map_point_cloud) {
  ros::Time time_current = ros::Time::now();
  //TODO check if T_odom_to_scanner can be reused
  try {

    // Move the global map to the scanner pose.
    T_scanner_to_map_ =
        PointMatcher_ros::eigenMatrixToDim<float>(
            PointMatcher_ros::transformListenerToEigenMatrix<float>(
                tf_listener_,
                parameters_.tf_map_frame,
                parameters_.sensor_frame,
                time_current
            ), map_point_cloud_->getHomogeneousDim());
    DP local_map =
        transformation_->compute(*new_map_point_cloud, T_scanner_to_map_
            .inverse());

    // Cut points in a radius of the parameter sensorMaxRange.
    radius_filter_->inPlaceFilter(local_map);

    icp_map_lock_.lock();
    // Update the transformation to the local map.
    this->T_local_map_to_map_ = T_scanner_to_map_;

    icp_.setMap(local_map);
    icp_map_lock_.unlock();
  } catch (...) {
    // Everything else.
    ROS_ERROR_STREAM("Unexpected exception... ignoring scan B");
    return;
  }
}

Mapper::DP *Mapper::updateMap(DP *new_point_cloud,
                              const PM::TransformationParameters
                              T_updated_scanner_to_map,
                              bool map_exists) {
  timer t;

  try {
    // Prepare empty field if not existing.
    if (new_point_cloud->descriptorExists("probabilityStatic") == false) {
      new_point_cloud->addDescriptor("probabilityStatic",
                                     PM::Matrix::Constant(1,
                                                          new_point_cloud->features.cols(),
                                                          parameters_.prior_static));
    }

    if (new_point_cloud->descriptorExists("probabilityDynamic") == false) {
      new_point_cloud->addDescriptor("probabilityDynamic",
                                     PM::Matrix::Constant(1,
                                                          new_point_cloud->features.cols(),
                                                          parameters_.prior_dyn));
    }

    if (!map_exists) {
      ROS_INFO_STREAM("[MAP] Initial map, only filtering points");
      *new_point_cloud =
          transformation_->compute(*new_point_cloud,
                                   T_updated_scanner_to_map);
      map_post_filters_.apply(*new_point_cloud);

      return new_point_cloud;
    }


    // Early out if no map modification is wanted.
    if (!parameters_.mapping) {
      ROS_INFO_STREAM("[MAP] Skipping modification of the map");
      return map_point_cloud_;
    }

    const int map_pts_count(map_point_cloud_->getNbPoints());
    const int read_pts_count(new_point_cloud->getNbPoints());
    const int dim = map_point_cloud_->getEuclideanDim();

    // Transform the global map in local coordinates.
    DP map_local_frame_cut = transformation_->compute(*map_point_cloud_,
                                                      T_updated_scanner_to_map
                                                          .inverse());
    // Perform density computation.
    map_local_frame_cut.concatenate(*new_point_cloud);

    // Build and populate NNS.
    std::shared_ptr<NNS> featureNNS;
    featureNNS.reset(NNS::create(map_local_frame_cut.features,
                                 map_local_frame_cut.features.rows() - 1,
                                 NNS::KDTREE_LINEAR_HEAP,
                                 NNS::TOUCH_STATISTICS));

    PM::Matches matches_overlap(
        Matches::Dists(1, read_pts_count),
        Matches::Ids(1, read_pts_count)
    );

    featureNNS->knn(new_point_cloud->features,
                    matches_overlap.ids,
                    matches_overlap.dists,
                    1,
                    0);

    DP no_overlap(new_point_cloud->createSimilarEmpty());

    int pts_out = 0;
    for (int i = 0; i < read_pts_count; ++i) {
      if (matches_overlap.dists(i) > parameters_.max_dist_new_point) {
        no_overlap.setColFrom(pts_out, *new_point_cloud, i);
        pts_out++;
      }
    }

    no_overlap.conservativeResize(pts_out);

    // Shrink the new PointCloud to the new information.
    *new_point_cloud = no_overlap;

    // Correct new points using ICP result.
    *new_point_cloud =
        transformation_->compute(*new_point_cloud, T_updated_scanner_to_map);

    // Merge point clouds to map.
    new_point_cloud->concatenate(*map_point_cloud_);
    map_post_filters_.apply(*new_point_cloud);
  } catch (DP::InvalidField e) {
    ROS_ERROR_STREAM(e.what());
    abort();
  } catch (const std::exception &exc) {
    std::cerr << exc.what();
    abort();
  }

  ROS_INFO_STREAM(
      "[TIME][MAP] New map available (" << new_point_cloud->features.cols()
                                        << " pts), update took " << t.elapsed()
                                        << " [s]");

  return new_point_cloud;
}

void Mapper::waitForMapBuildingCompleted() {
#if BOOST_VERSION >= 104100
  if (map_building_in_progress_) {
    // we wait for now, in future we should kill it
    map_building_future_.wait();
    map_building_in_progress_ = false;
  }
#endif // BOOST_VERSION >= 104100
}

bool Mapper::getPointMap(map_msgs::GetPointMap::Request &req,
                         map_msgs::GetPointMap::Response &res) {
  if (!map_point_cloud_)
    return false;

  // FIXME: do we need a mutex here?
  res.map =
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*map_point_cloud_,
                                                         parameters_.map_frame,
                                                         ros::Time::now());
  return true;
}

bool Mapper::saveMap(map_msgs::SaveMap::Request &req,
                     map_msgs::SaveMap::Response &res) {
  if (!map_point_cloud_)
    return false;

  try {
    map_point_cloud_->save(req.filename.data + ".vtk");
    PointMatcherIO<float>::savePCD(*map_point_cloud_, req.filename.data + ".pcd");

  } catch (const std::runtime_error &e) {
    ROS_ERROR_STREAM("Unable to save: " << e.what());
    return false;
  }

  ROS_INFO_STREAM("[MAP] saved at " << req.filename.data << " with "
                                    << map_point_cloud_->features.cols()
                                    << " points.");
  return true;
}

bool Mapper::loadMap(ethzasl_icp_mapper::LoadMap::Request &req,
                     ethzasl_icp_mapper::LoadMap::Response &res) {
  waitForMapBuildingCompleted();

  DP *cloud(new DP(DP::load(req.filename.data)));

  // Print new map information.
  const int dim = cloud->features.rows();
  const int nbPts = cloud->features.cols();
  ROS_INFO_STREAM(
      "[MAP] Loading " << dim - 1 << "D point cloud (" << req.filename.data
                       << ") with " << nbPts << " points.");

  ROS_INFO_STREAM("  With descriptors:");
  for (int i = 0; i < cloud->descriptorLabels.size(); i++) {
    ROS_INFO_STREAM("    - " << cloud->descriptorLabels[i].text);
  }

  T_local_map_to_map_ = PM::TransformationParameters::Identity(dim, dim);
  T_scanner_to_map_ = PM::TransformationParameters::Identity(dim, dim);
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

  publish_lock_.lock();
  // WARNING: this will break in 2D.
  T_local_map_to_map_ = PM::TransformationParameters::Identity(4, 4);
  T_scanner_to_map_ = PM::TransformationParameters::Identity(4, 4);
  publish_lock_.unlock();

  icp_.clearMap();

  return true;
}

bool Mapper::correctPose(ethzasl_icp_mapper::CorrectPose::Request &req,
                         ethzasl_icp_mapper::CorrectPose::Response &res) {
  publish_lock_.lock();
  const int dim = map_point_cloud_->getHomogeneousDim();

  try {

    T_scanner_to_map_ =
        PointMatcher_ros::eigenMatrixToDim<float>(
            PointMatcher_ros::transformListenerToEigenMatrix<float>(
                tf_listener_,
                parameters_.tf_map_frame,
                parameters_.sensor_frame,
                ros::Time::now()
            ), dim);

    updateIcpMap(map_point_cloud_);
  } catch (...) {
    // everything else
    publish_lock_.unlock();
    ROS_ERROR_STREAM("Unexpected exception... ignoring scan C");
    return false;
  }

  publish_lock_.unlock();
  return true;
}

bool Mapper::setMode(ethzasl_icp_mapper::SetMode::Request &req,
                     ethzasl_icp_mapper::SetMode::Response &res) {
  // Impossible states.
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
  pose_pub_.publish(req.transform);
  return true;
}

bool Mapper::loadPublishedMap(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res) {
  parameters_.map_trigger = true;
}

bool Mapper::getBoundedMap(ethzasl_icp_mapper::GetBoundedMap::Request &req,
                           ethzasl_icp_mapper::GetBoundedMap::Response &res) {
  if (!map_point_cloud_)
    return false;

  const float max_x = req.topRightCorner.x;
  const float max_y = req.topRightCorner.y;
  const float max_z = req.topRightCorner.z;

  const float min_x = req.bottomLeftCorner.x;
  const float min_y = req.bottomLeftCorner.y;
  const float min_z = req.bottomLeftCorner.z;

  Eigen::Affine3d eigen_T;
  tf::poseMsgToEigen(req.mapCenter, eigen_T);
  Eigen::MatrixXf T = eigen_T.matrix().inverse().cast<float>();

  T = transformation_->correctParameters(T);


  // FIXME: do we need a mutex here?
  const DP centered_point_cloud = transformation_->compute(*map_point_cloud_,
                                                           T);
  DP cut_point_cloud = centered_point_cloud.createSimilarEmpty();

  int new_pt_count = 0;
  for (int i = 0; i < centered_point_cloud.features.cols(); i++) {
    const float x = centered_point_cloud.features(0, i);
    const float y = centered_point_cloud.features(1, i);
    const float z = centered_point_cloud.features(2, i);

    if (x < max_x && x > min_x &&
        y < max_y && y > min_y &&
        z < max_z && z > min_z) {
      cut_point_cloud.setColFrom(new_pt_count, centered_point_cloud, i);
      new_pt_count++;
    }
  }

  ROS_INFO_STREAM("Extract " << new_pt_count << " points from the map");

  cut_point_cloud.conservativeResize(new_pt_count);
  cut_point_cloud = transformation_->compute(cut_point_cloud, T.inverse());


  // Send the resulting point cloud in ROS format.
  res.boundedMap = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
      cut_point_cloud,
      parameters_.map_frame,
      ros::Time::now());
  return true;
}

void Mapper::loadExternalParameters() {

  // Load configs.
  string config_file_name;
  if (ros::param::get("~icpConfig", config_file_name)) {
    ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      icp_.loadFromYaml(ifs);
    } else {
      ROS_ERROR_STREAM(
          "Cannot load ICP config from YAML file " << config_file_name);
      icp_.setDefault();
    }
  } else {
    ROS_INFO_STREAM("No ICP config file given, using default");
    icp_.setDefault();
  }

  if (ros::param::get("~inputFiltersConfig", config_file_name)) {
    ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      input_filters_ = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM(
          "Cannot load input filters config from YAML file "
              << config_file_name);
    }
  } else {
    ROS_INFO_STREAM(
        "No input filters config file given, not using these filters");
  }

  if (ros::param::get("~mapPreFiltersConfig", config_file_name)) {
    ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      map_pre_filters_ = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM("Cannot load map pre-filters config from YAML file "
                           << config_file_name);
    }
  } else {
    ROS_INFO_STREAM(
        "No map pre-filters config file given, not using these filters");
  }

  if (ros::param::get("~mapPostFiltersConfig", config_file_name)) {
    ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      map_post_filters_ = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file "
                           << config_file_name);
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
