#include <pointmatcher_ros/get_params_from_server.h>

#include "ethzasl_icp_mapper/mapper_parameters.h"

namespace mapper {

MapperParameters::MapperParameters() :
    use_const_motion_model(getParam<bool>("useConstMotionModel", false)),
    localizing(getParam<bool>("localizing", true)),
    mapping(getParam<bool>("mapping", true)),
    min_reading_point_count(getParam<int>("minReadingPointCount", 2000)),
    min_map_point_count(getParam<int>("minMapPointCount", 500)),
    input_queue_size(getParam<int>("inputQueueSize", 10)),
    min_overlap(getParam<double>("minOverlap", 0.5)),
    max_overlap_to_merge(getParam<double>("maxOverlapToMerge", 0.9)),
    tf_refresh_period(getParam<double>("tfRefreshPeriod", 0.01)),
    sensor_frame(getParam<std::string>("sensor_frame", "")),
    odom_frame(getParam<std::string>("odom_frame", "odom")),
    map_frame(getParam<std::string>("map_frame", "world")),
    tf_map_frame(getParam<std::string>("tf_map_frame", "map")),
    lidar_frame(getParam<std::string>("lidar_frame", "lidar")),
    vtk_final_map_name(getParam<std::string>("vtkFinalMapName", "finalMap.vtk")),
    prior_dyn(getParam<double>("priorDyn", 0.5)),
    prior_static(1. - prior_dyn),
    max_angle(getParam<double>("maxAngle", 0.02)),
    eps_a(getParam<double>("eps_a", 0.05)),
    eps_d(getParam<double>("eps_d", 0.02)),
    alpha(getParam<double>("alpha", 0.99)),
    beta(getParam<double>("beta", 0.99)),
    max_dyn(getParam<double>("maxDyn", 0.95)),
    max_dist_new_point(pow(getParam<double>("maxDistNewPoint", 0.1), 2)),
    sensor_max_range(getParam<double>("sensorMaxRange", 80.0)),
    map_trigger(false),
    use_logger(getParam<bool>("useROSLogger", false)),
    subscribe_cloud(getParam<bool>("subscribe_cloud", true)),
    subscribe_map(getParam<bool>("subscribe_map", true))
{
  // Ensure proper states.
  if (localizing == false)
    mapping = false;
  if (mapping == true)
    localizing = true;
}

MapperParameters::~MapperParameters(){}

}