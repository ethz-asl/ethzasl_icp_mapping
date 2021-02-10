#ifndef ETHZASL_ICP_MAPPER_DYNAMIC_MAPPER_H
#define ETHZASL_ICP_MAPPER_DYNAMIC_MAPPER_H

#include <boost/thread.hpp>
#include <boost/version.hpp>
#if BOOST_VERSION >= 104100
#include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100
#include <fstream>

#include <eigen_conversions/eigen_msg.h>
#include <map_msgs/GetPointMap.h>
#include <map_msgs/SaveMap.h>
#include <nabo/nabo.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Timer.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/ros_logger.h>
#include <pointmatcher_ros/transform.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "ethzasl_icp_mapper/LoadMap.h"
#include "ethzasl_icp_mapper/CorrectPose.h"
#include "ethzasl_icp_mapper/SetMode.h"
#include "ethzasl_icp_mapper/GetMode.h"
#include "ethzasl_icp_mapper/InitialTransform.h"
#include "ethzasl_icp_mapper/GetBoundedMap.h"
#include "ethzasl_icp_mapper/mapper_parameters.h"

namespace mapper {

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;

class Mapper {
  typedef PM::DataPoints DP;
  typedef PM::Matches Matches;

  typedef typename Nabo::NearestNeighbourSearch<float> NNS;
  typedef typename NNS::SearchType NNSearchType;

 public:
  Mapper(ros::NodeHandle &n, ros::NodeHandle &pn);
  ~Mapper();

 protected:
  void gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in);
  void gotMap(const sensor_msgs::PointCloud2 &cloud_msg_in);
  void processCloud(unique_ptr<DP> cloud,
                    const std::string &scanner_frame,
                    const ros::Time &stamp,
                    uint32_t seq, bool new_map);
  void processNewMapIfAvailable();
  void setMap(DP *new_point_cloud);
  DP *updateMap(DP *new_point_cloud,
                const PM::TransformationParameters T_updated_scanner_to_map,
                bool map_exists);
  void waitForMapBuildingCompleted();
  void updateIcpMap(const DP *new_map_point_cloud);
  void loadExternalParameters();

  // Services
  bool getPointMap(map_msgs::GetPointMap::Request &req,
                   map_msgs::GetPointMap::Response &res);
  bool saveMap(map_msgs::SaveMap::Request &req,
               map_msgs::SaveMap::Response &res);
  bool loadMap(ethzasl_icp_mapper::LoadMap::Request &req,
               ethzasl_icp_mapper::LoadMap::Response &res);
  bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool correctPose(ethzasl_icp_mapper::CorrectPose::Request &req,
                   ethzasl_icp_mapper::CorrectPose::Response &res);
  bool setMode(ethzasl_icp_mapper::SetMode::Request &req,
               ethzasl_icp_mapper::SetMode::Response &res);
  bool getMode(ethzasl_icp_mapper::GetMode::Request &req,
               ethzasl_icp_mapper::GetMode::Response &res);
  bool getBoundedMap(ethzasl_icp_mapper::GetBoundedMap::Request &req,
                     ethzasl_icp_mapper::GetBoundedMap::Response &res);
  bool reloadallYaml(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &res);
  bool initialTransform(ethzasl_icp_mapper::InitialTransform::Request &req,
                        ethzasl_icp_mapper::InitialTransform::Response &res);
  bool loadPublishedMap(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res);

 private:
  ros::NodeHandle &n;
  ros::NodeHandle &pn;

  // Subscribers
  ros::Subscriber cloud_sub_;
  ros::Subscriber map_sub_;

  // Publishers
  ros::Publisher map_pub_;
  ros::Publisher scan_pub_;
  ros::Publisher outlier_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher odom_base_pub_;
  ros::Publisher odom_error_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher pose_base_pub_;

  // Services
  ros::ServiceServer get_point_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer correct_pose_srv_;
  ros::ServiceServer set_mode_srv_;
  ros::ServiceServer get_mode_srv_;
  ros::ServiceServer get_bounded_map_srv_;
  ros::ServiceServer reload_all_yaml_srv_;
  ros::ServiceServer initial_transform_srv_;
  ros::ServiceServer load_published_map_srv_;

  // Time
  ros::Time map_creation_time_;
  ros::Time last_poin_cloud_time_;
  uint32_t last_point_cloud_seq_;

  // libpointmatcher
  PM::DataPointsFilters input_filters_;
  PM::DataPointsFilters map_pre_filters_;
  PM::DataPointsFilters map_post_filters_;
  PM::DataPoints *map_point_cloud_;
  PM::ICPSequence icp_;
  shared_ptr<PM::Transformation> transformation_;
  shared_ptr<PM::DataPointsFilter> radius_filter_;

  // multi-threading mapper
#if BOOST_VERSION >= 104100
  typedef boost::packaged_task<PM::DataPoints *> MapBuildingTask;
  typedef boost::unique_future<PM::DataPoints *> MapBuildingFuture;
  boost::thread map_building_thread_;
  MapBuildingTask map_building_task_;
  MapBuildingFuture map_building_future_;
  bool map_building_in_progress_;
#endif // BOOST_VERSION >= 104100

  MapperParameters parameters_;
  int odom_received_;
  PM::TransformationParameters T_scanner_to_odom_;
  PM::TransformationParameters T_odom_to_map_;
  boost::thread publish_thread_;
  boost::mutex publish_lock_;
  boost::mutex icp_map_lock_;
  int scan_counter_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
};

}

#endif // ETHZASL_ICP_MAPPER_DYNAMIC_MAPPER_H