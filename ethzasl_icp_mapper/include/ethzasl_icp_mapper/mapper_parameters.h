#ifndef ETHZASL_ICP_MAPPER_MAPPER_PARAMETERS_H
#define ETHZASL_ICP_MAPPER_MAPPER_PARAMETERS_H

#include <boost/version.hpp>
#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
#include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100
#include <fstream>

namespace mapper {

struct MapperParameters {

  MapperParameters();
  ~MapperParameters();

  // Parameters
  bool use_const_motion_model;
  bool localizing;
  bool mapping;
  int min_reading_point_count;
  int min_map_point_count;
  int input_queue_size;
  double min_overlap;
  double max_overlap_to_merge;
  double tf_refresh_period;
  bool map_trigger;
  bool use_logger;
  bool subscribe_cloud;
  bool subscribe_map;
  std::string sensor_frame;
  std::string base_frame;
  std::string odom_frame;
  std::string map_frame;
  std::string tf_map_frame;
  std::string lidar_frame;
  std::string vtk_final_map_name;
  int skip_frames;

  // Parameters for dynamic filtering
  const float prior_dyn;
  const float prior_static;
  const float max_angle;
  const float eps_a;
  const float eps_d;
  const float alpha;
  const float beta;
  const float max_dyn;
  const float max_dist_new_point;
  const float sensor_max_range;
};

}

#endif // ETHZASL_ICP_MAPPER_MAPPER_PARAMETERS_H