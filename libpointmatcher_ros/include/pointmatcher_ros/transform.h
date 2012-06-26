#ifndef __POINTMATCHER_ROS_TRANSFORM_H
#define __POINTMATCHER_ROS_TRANSFORM_H

#include "pointmatcher/PointMatcher.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Eigen"

namespace ros
{
	struct Time;
};
namespace tf
{
	struct TransformListener;
	struct StampedTransform;
};

namespace PointMatcher_ros
{
	// from tf/ROS
	
	template<typename T>
	typename PointMatcher<T>::TransformationParameters transformListenerToEigenMatrix(const tf::TransformListener &listener, const std::string& child, const std::string& parent, const ros::Time& stamp);
	
	template<typename T>
	typename PointMatcher<T>::TransformationParameters odomMsgToEigenMatrix(const nav_msgs::Odometry& odom);
	
	// to tf/ROS
	
	template<typename T>
	nav_msgs::Odometry eigenMatrixToOdomMsg(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& frame_id, const ros::Time& stamp);
	
	template<typename T>
	tf::StampedTransform eigenMatrixToStampedTransform(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& child, const std::string& parent, const ros::Time& stamp);
	
}; // PointMatcher_ros

#endif //__POINTMATCHER_ROS_TRANSFORM_H
