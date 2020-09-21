#include <ros/ros.h>
#include <thread>

#include "map_interface/map_interface.h"

using namespace map_interface;

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_interface_node");
  ros::NodeHandle node_handle("~");

  MapInterface map_interface(node_handle);
  std::string filename;
  node_handle.getParam("/map_file_pcd", filename);
  map_interface.load(filename);
  std::thread publish_point_cloud_thread
      (&MapInterface::publishPointCloudThread, &map_interface);

  try {
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }

  publish_point_cloud_thread.join();

  return 0;
}
