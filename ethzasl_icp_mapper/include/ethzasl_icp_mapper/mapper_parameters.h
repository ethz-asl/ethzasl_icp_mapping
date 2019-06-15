#ifndef ETHZASL_ICP_MAPPER_MAPPER_PARAMETERS_H
#define ETHZASL_ICP_MAPPER_MAPPER_PARAMETERS_H

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

#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

namespace mapper {

typedef PointMatcher<float> PM;

struct MapperParameters {
  typedef PM::DataPoints DP;

  MapperParameters();
  ~MapperParameters();

  // Parameters
  bool useConstMotionModel;
  bool localizing;
  bool mapping;
  int minReadingPointCount;
  int minMapPointCount;
  int inputQueueSize;
  double minOverlap;
  double maxOverlapToMerge;
  double tfRefreshPeriod;
  bool cad_trigger;
  bool use_logger;
  bool subscribe_cloud;
  bool subscribe_cad;
  std::string sensorFrame;
  std::string odomFrame;
  std::string mapFrame;
  std::string tfMapFrame;
  std::string lidarFrame;
  std::string vtkFinalMapName; //!< name of the final vtk map

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
  const float eps;
};

}

#endif // ETHZASL_ICP_MAPPER_MAPPER_PARAMETERS_H