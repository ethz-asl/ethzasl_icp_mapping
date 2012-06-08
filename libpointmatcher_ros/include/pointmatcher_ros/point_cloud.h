#ifndef __POINTMATCHER_ROS_POINT_CLOUD_H
#define __POINTMATCHER_ROS_POINT_CLOUD_H

#include "pointmatcher/PointMatcher.h"
#include "sensor_msgs/PointCloud2.h"

namespace ros
{
	struct Time;
};

namespace PointMatcher_ros
{
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg);
	
	template<typename T>
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);
}; // PointMatcher_ros

#endif //__POINTMATCHER_ROS_POINT_CLOUD_H
