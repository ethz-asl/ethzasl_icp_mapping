#ifndef ETHZASL_ICP_MAPPER_DYNAMIC_MAPPER_H
#define ETHZASL_ICP_MAPPER_DYNAMIC_MAPPER_H

#include <fstream>

#include <boost/version.hpp>
#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
#include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "nabo/nabo.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/ros_logger.h"

#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

// Services

#include "std_srvs/Empty.h"
#include "map_msgs/GetPointMap.h"
#include "map_msgs/SaveMap.h"
#include "ethzasl_icp_mapper/LoadMap.h"
#include "ethzasl_icp_mapper/CorrectPose.h"
#include "ethzasl_icp_mapper/SetMode.h"
#include "ethzasl_icp_mapper/GetMode.h"
#include "ethzasl_icp_mapper/InitialTransform.h"
#include "ethzasl_icp_mapper/GetBoundedMap.h" // FIXME: should that be moved to map_msgs?

namespace mapper {

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;

class Mapper {
  typedef PM::DataPoints DP;
  typedef PM::Matches Matches;

  typedef typename Nabo::NearestNeighbourSearch<float> NNS;
  typedef typename NNS::SearchType NNSearchType;

  ros::NodeHandle &n;
  ros::NodeHandle &pn;

  // Subscribers
  ros::Subscriber cloudSub;
  ros::Subscriber cadSub;

  // Publishers
  ros::Publisher mapPub;
  ros::Publisher scanPub;
  ros::Publisher outlierPub;
  ros::Publisher odomPub;
  ros::Publisher odomErrorPub;
  ros::Publisher posePub;

  // Services
  ros::ServiceServer getPointMapSrv;
  ros::ServiceServer saveMapSrv;
  ros::ServiceServer loadMapSrv;
  ros::ServiceServer resetSrv;
  ros::ServiceServer correctPoseSrv;
  ros::ServiceServer setModeSrv;
  ros::ServiceServer getModeSrv;
  ros::ServiceServer getBoundedMapSrv;
  ros::ServiceServer reloadAllYamlSrv;
  ros::ServiceServer initialTransformSrv;
  ros::ServiceServer loadPublishedMapSrv;

  // Time
  ros::Time mapCreationTime;
  ros::Time lastPoinCloudTime;
  uint32_t lastPointCloudSeq;

  // libpointmatcher
  PM::DataPointsFilters inputFilters;
  PM::DataPointsFilters mapPreFilters;
  PM::DataPointsFilters mapPostFilters;
  PM::DataPoints *mapPointCloud;
  PM::ICPSequence icp;
  shared_ptr<PM::Transformation> transformation;
  shared_ptr<PM::DataPointsFilter> radiusFilter;

  // multi-threading mapper
#if BOOST_VERSION >= 104100
  typedef boost::packaged_task<PM::DataPoints *> MapBuildingTask;
  typedef boost::unique_future<PM::DataPoints *> MapBuildingFuture;
  boost::thread mapBuildingThread;
  MapBuildingTask mapBuildingTask;
  MapBuildingFuture mapBuildingFuture;
  bool mapBuildingInProgress;
#endif // BOOST_VERSION >= 104100

  // Parameters
  bool useConstMotionModel;
  bool localizing;
  bool mapping;
  int minReadingPointCount;
  int minMapPointCount;
  int inputQueueSize;
  double minOverlap;
  double maxOverlapToMerge;
  double
      tfRefreshPeriod;  //!< if set to zero, tf will be publish at the rate of the incoming point cloud messages
  bool cad_trigger;
  int odom_received;
  string sensorFrame;
  string odomFrame;
  string mapFrame;
  string tfMapFrame;
  string lidarFrame;
  string vtkFinalMapName; //!< name of the final vtk map

  // Parameters for dynamic filtering
  const float
      priorDyn; //!< ratio. Prior to be dynamic when a new point is added
  const float
      priorStatic; //!< ratio. Prior to be static when a new point is added
  const float maxAngle; //!< in rad. Openning angle of a laser beam
  const float eps_a; //!< ratio. Error proportional to the laser distance
  const float eps_d; //!< in meter. Fix error on the laser distance
  const float
      alpha; //!< ratio. Propability of staying static given that the point was dynamic
  const float
      beta; //!< ratio. Propability of staying dynamic given that the point was static
  const float maxDyn; //!< ratio. Threshold for which a point will stay dynamic
  const float
      maxDistNewPoint; //!< in meter. Distance at which a new point will be added in the global map.
  const float
      sensorMaxRange; //!< in meter. Maximum reading distance of the laser. Used to cut the global map before matching.

  PM::TransformationParameters T_localMap_to_map;
  PM::TransformationParameters T_scanner_to_map;
  boost::thread publishThread;
  boost::mutex publishLock;
  boost::mutex icpMapLock;
  ros::Time publishStamp;

  tf::TransformListener tfListener;
  tf::TransformBroadcaster tfBroadcaster;

  const float eps;

 public:
  Mapper(ros::NodeHandle &n, ros::NodeHandle &pn);
  ~Mapper();

 protected:
  void gotCloud(const sensor_msgs::PointCloud2 &cloudMsgIn);
  void gotCAD(const sensor_msgs::PointCloud2 &cloudMsgIn);
  void processCloud(unique_ptr<DP> cloud,
                    const std::string &scannerFrame,
                    const ros::Time &stamp,
                    uint32_t seq);
  void processNewMapIfAvailable();
  void setMap(DP *newPointCloud);
  DP *updateMap(DP *newPointCloud,
                const PM::TransformationParameters T_updatedScanner_to_map,
                bool mapExists);
  void waitForMapBuildingCompleted();
  void updateIcpMap(const DP *newMapPointCloud);

//  void publishLoop(double publishPeriod);
//  void publishTransform();
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
};

}

#endif // ETHZASL_ICP_MAPPER_DYNAMIC_MAPPER_H