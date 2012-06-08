#ifndef __POINTMATCHER_ROS_POINT_CLOUD_H
#define __POINTMATCHER_ROS_POINT_CLOUD_H

#include "pointmatcher/PointMatcher.h"
#include "nav_msgs/Odometry.h"

namespace ros
{
	struct Time;
};
namespace tf
{
	struct TransformListener;
};

namespace PointMatcher_ros
{
	template<typename T>
	typename PointMatcher<T>::TransformationParameters transformListenerToEigenMatrix(const tf::TransformListener &listener, const std::string& child, const std::string& parent, const ros::Time& stamp);
	
	template<typename T>
	nav_msgs::Odometry eigenMatrixToOdomMsg(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& frame_id, const ros::Time& stamp);
	
	template<typename T>
	typename PointMatcher<T>::TransformationParameters odomMsgToEigenMatrix(const nav_msgs::Odometry& odom);
}; // PointMatcher_ros

#endif //__POINTMATCHER_ROS_POINT_CLOUD_H
