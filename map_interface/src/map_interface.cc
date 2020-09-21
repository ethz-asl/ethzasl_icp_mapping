#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "map_interface/map_interface.h"

namespace map_interface {

MapInterface::MapInterface(ros::NodeHandle& n) : cloud_(new pcl::PointCloud<pcl::PointXYZ>), nh_(n), iterator_(0) {
  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_model", 100);
  point_cloud_sparse_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_model_sparse", 100);
};

MapInterface::~MapInterface() {};

void MapInterface::load(const std::string &filename) {
  // Loading of mesh data.
  pcl::io::loadPCDFile(filename, *cloud_);
  cloud_->header.frame_id = "marker2";
  // Convert to PCLPointCloud2.
  pcl::PCLPointCloud2 pcl_point_cloud_2;
  pcl::toPCLPointCloud2(*cloud_, pcl_point_cloud_2);
  // Convert to sensor_msgs::PointCloud2.
  pcl_conversions::fromPCL(pcl_point_cloud_2, map_cloud_msg_);
  // Apply frame to msg.
  map_cloud_msg_.header.frame_id = cloud_->header.frame_id;
  // Produce downsampled cloud for visualization
  pcl::PCLPointCloud2 pcl_point_cloud_sparse;
  pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
  filter.setInputCloud(boost::make_shared<pcl::PCLPointCloud2>(pcl_point_cloud_2));
  filter.setLeafSize(0.2f, 0.2f, 0.2f);
  filter.filter(pcl_point_cloud_sparse);
  pcl_conversions::fromPCL(pcl_point_cloud_sparse, map_cloud_msg_sparse_);
  map_cloud_msg_sparse_.header.frame_id = cloud_->header.frame_id;
}

void MapInterface::publishPointCloudThread() {
    ros::Rate thread_rate(0.5);
    while (ros::ok()) {

      // Publish dense point cloud
      map_cloud_msg_.header.stamp = ros::Time::now();
      map_cloud_msg_.header.seq = iterator_++;
      point_cloud_pub_.publish(map_cloud_msg_);
      // Publish sparse point cloud
      map_cloud_msg_sparse_.header.stamp = ros::Time::now();
      map_cloud_msg_sparse_.header.seq = iterator_;
      point_cloud_sparse_pub_.publish(map_cloud_msg_sparse_);

      thread_rate.sleep();
    }
}


} //namespace map_interface
