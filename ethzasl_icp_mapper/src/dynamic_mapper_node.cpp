#include <ros/ros.h>

#include "ethzasl_icp_mapper/dynamic_mapper.h"

// Main function supporting the Mapper class
int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  mapper::Mapper mapper(n, pn);
  ros::spin();

  return 0;
}