#include "pointmatcher_ros/transform.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"
#include "ros/ros.h"

namespace PointMatcher_ros
{
	template<typename T>
	typename PointMatcher<T>::TransformationParameters transformListenerToEigenMatrix(const tf::TransformListener &listener, const std::string& target, const std::string& source, const ros::Time& stamp)
	{
		typedef typename PointMatcher<T>::TransformationParameters TransformationParameters;
		
		tf::StampedTransform stampedTr;
		listener.waitForTransform(target, source, stamp, ros::Duration(1));
		listener.lookupTransform(target, source, stamp, stampedTr);
		
		Eigen::Affine3d eigenTr;
		tf::TransformTFToEigen(stampedTr, eigenTr);
		return eigenTr.matrix().cast<T>();
	}
	
	template
	PointMatcher<float>::TransformationParameters transformListenerToEigenMatrix<float>(const tf::TransformListener &listener, const std::string& target, const std::string& source, const ros::Time& stamp);
	template
	PointMatcher<double>::TransformationParameters transformListenerToEigenMatrix<double>(const tf::TransformListener &listener, const std::string& target, const std::string& source, const ros::Time& stamp);

	
	template<typename T>
	typename PointMatcher<T>::TransformationParameters odomMsgToEigenMatrix(const nav_msgs::Odometry& odom)
	{
		Eigen::Affine3d eigenTr;
		tf::poseMsgToEigen(odom.pose.pose, eigenTr);
		return eigenTr.matrix().cast<T>();
	}
	
	template
	PointMatcher<float>::TransformationParameters odomMsgToEigenMatrix<float>(const nav_msgs::Odometry& odom);
	template
	PointMatcher<double>::TransformationParameters odomMsgToEigenMatrix<double>(const nav_msgs::Odometry& odom);
	
	
	template<typename T>
	nav_msgs::Odometry eigenMatrixToOdomMsg(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& frame_id, const ros::Time& stamp)
	{
		nav_msgs::Odometry odom;
		odom.header.stamp = stamp;
		odom.header.frame_id = frame_id;
		
		// Fill pose
		const Eigen::Affine3d eigenTr(
			Eigen::Matrix4d(
				eigenMatrixToDim<double>(
					inTr.template cast<double>(), 4
				)
			)
		);
		tf::poseEigenToMsg(eigenTr, odom.pose.pose);

		// Fill velocity, TODO: find proper computation from delta poses to twist
		//odom.child_frame_id = cloudMsgIn.header.frame_id;
		odom.twist.covariance[0+0*6] = -1;
		odom.twist.covariance[1+1*6] = -1;
		odom.twist.covariance[2+2*6] = -1;
		odom.twist.covariance[3+3*6] = -1;
		odom.twist.covariance[4+4*6] = -1;
		odom.twist.covariance[5+5*6] = -1;

		return odom;
	}
	
	template
	nav_msgs::Odometry eigenMatrixToOdomMsg<float>(const typename PointMatcher<float>::TransformationParameters& inTr, const std::string& frame_id, const ros::Time& stamp);
	template
	nav_msgs::Odometry eigenMatrixToOdomMsg<double>(const typename PointMatcher<double>::TransformationParameters& inTr, const std::string& frame_id, const ros::Time& stamp);
	

	template<typename T>
	tf::StampedTransform eigenMatrixToStampedTransform(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& target, const std::string& source, const ros::Time& stamp)
	{
		tf::Transform tfTr;
		const Eigen::Affine3d eigenTr(
			Eigen::Matrix4d(
				eigenMatrixToDim<double>(
					inTr.template cast<double>(), 4
				)
			)
		);
		tf::TransformEigenToTF(eigenTr, tfTr);
		return tf::StampedTransform(tfTr, stamp, target, source);
	}
	
	template
	tf::StampedTransform eigenMatrixToStampedTransform<float>(const typename PointMatcher<float>::TransformationParameters& inTr, const std::string& target, const std::string& source, const ros::Time& stamp);
	template
	tf::StampedTransform eigenMatrixToStampedTransform<double>(const typename PointMatcher<double>::TransformationParameters& inTr, const std::string& target, const std::string& source, const ros::Time& stamp);
	
	template<typename T>
	typename PointMatcher<T>::TransformationParameters eigenMatrixToDim(const typename PointMatcher<T>::TransformationParameters& matrix, int dimp1)
	{
		typedef typename PointMatcher<T>::TransformationParameters M;
		assert(matrix.rows() == matrix.cols());
		assert((matrix.rows() == 3) || (matrix.rows() == 4));
		assert((dimp1 == 3) || (dimp1 == 4));
		
		if (matrix.rows() == dimp1)
			return matrix;
		
		M out(M::Identity(dimp1,dimp1));
		out.topLeftCorner(2,2) = matrix.topLeftCorner(2,2);
		out.topRightCorner(2,1) = matrix.topRightCorner(2,1);
		return out;
	}
	
	template
	typename PointMatcher<float>::TransformationParameters eigenMatrixToDim<float>(const typename PointMatcher<float>::TransformationParameters& matrix, int dimp1);
	template
	typename PointMatcher<double>::TransformationParameters eigenMatrixToDim<double>(const typename PointMatcher<double>::TransformationParameters& matrix, int dimp1);
}; // PointMatcher_ros
