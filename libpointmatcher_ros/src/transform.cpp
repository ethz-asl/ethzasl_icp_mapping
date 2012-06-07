#include "libpointmatcher_ros/transforms.h"

PM::TransformationParameters PointMatcher_ros::transformListenerToEigenMatrix(const tf::TransformListener &listener, const std::string child, const std::string parent, ros::Time stamp)
{
	PM::TransformationParameters T = PM::TransformationParameters::Identity(4,4);

	tf::StampedTransform transform;
	
	try
	{
		listener.lookupTransform(parent, child, stamp, transform);
		//TODO: is there a better way to get a matrix?
		T(0,3) = transform.getOrigin().x();
		T(1,3) = transform.getOrigin().y();
		T(2,3) = transform.getOrigin().z();
		btMatrix3x3 bt = transform.getBasis();
		for(int i=0; i < 3; i++)
			for(int j=0; j < 3; j++)
				T(i,j) = bt[i][j];
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	return T;
}

nav_msgs::Odometry PointMatcher_ros::eigenMatrixToOdomMsg(const PM::TransformationParameters T, const std::string frame_id, ros::Time stamp)
{
	nav_msgs::Odometry odom;
	odom.header.stamp = stamp;
	odom.header.frame_id = frame_id;

	tf::Transform t;
	t.setOrigin(tf::Vector3(T(0,3), T(1,3), T(2,3)));
	t.setBasis(btMatrix3x3(
		T(0,0), T(0,1),T(0,2),
		T(1,0), T(1,1),T(1,2),
		T(2,0), T(2,1),T(2,2))
	);

	// Fill pose
	tf::poseTFToMsg(t, odom.pose.pose);

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

PM::TransformationParameters PointMatcher_ros::odomMsgToEigenMatrix(const nav_msgs::Odometry odom)
{
	PM::TransformationParameters T = PM::TransformationParameters::Identity(4,4);
	tf::Transform t;
	
	tf::poseMsgToTF(odom.pose.pose, t);

	T(0,3) = t.getOrigin().x();
	T(1,3) = t.getOrigin().y();
	T(2,3) = t.getOrigin().z();
	btMatrix3x3 bt = t.getBasis();
	for(int i=0; i < 3; i++)
		for(int j=0; j < 3; j++)
			T(i,j) = bt[i][j];

	return T;
}
