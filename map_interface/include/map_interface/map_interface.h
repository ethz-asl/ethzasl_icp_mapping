#ifndef MAP_INTERFACE_MAP_INTERFACE_H_
#define MAP_INTERFACE_MAP_INTERFACE_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>


namespace map_interface {
class MapInterface {
 public:
  MapInterface(ros::NodeHandle &n);
  ~MapInterface();

  void load(const std::string &filename);

  void publishPointCloudThread();
 private:
  ros::Publisher point_cloud_pub_;
  ros::Publisher point_cloud_sparse_pub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  ros::NodeHandle &nh_;
  int iterator_;
  sensor_msgs::PointCloud2 map_cloud_msg_;
  sensor_msgs::PointCloud2 map_cloud_msg_sparse_;
};
} //namespace map_interface

#endif /* MAP_INTERFACE_MAP_INTERFACE_H_ */
