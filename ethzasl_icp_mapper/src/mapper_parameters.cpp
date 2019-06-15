#include <pointmatcher_ros/get_params_from_server.h>

#include "ethzasl_icp_mapper/mapper_parameters.h"

namespace mapper {

MapperParameters::MapperParameters() :
    useConstMotionModel(getParam<bool>("useConstMotionModel", false)),
    localizing(getParam<bool>("localizing", true)),
    mapping(getParam<bool>("mapping", true)),
    minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
    minMapPointCount(getParam<int>("minMapPointCount", 500)),
    inputQueueSize(getParam<int>("inputQueueSize", 10)),
    minOverlap(getParam<double>("minOverlap", 0.5)),
    maxOverlapToMerge(getParam<double>("maxOverlapToMerge", 0.9)),
    tfRefreshPeriod(getParam<double>("tfRefreshPeriod", 0.01)),
    sensorFrame(getParam<std::string>("sensor_frame", "")),
    odomFrame(getParam<std::string>("odom_frame", "odom")),
    mapFrame(getParam<std::string>("map_frame", "world")),
    tfMapFrame(getParam<std::string>("tf_map_frame", "map")),
    lidarFrame(getParam<std::string>("lidar_frame", "lidar")),
    vtkFinalMapName(getParam<std::string>("vtkFinalMapName", "finalMap.vtk")),
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
    eps(0.0001),
    cad_trigger(false),
    use_logger(getParam<bool>("useROSLogger", false)),
    subscribe_cloud(getParam<bool>("subscribe_cloud", true)),
    subscribe_cad(getParam<bool>("subscribe_cad", true))
{
  // Ensure proper states
  if (localizing == false)
    mapping = false;
  if (mapping == true)
    localizing = true;
}

MapperParameters::~MapperParameters(){}

}