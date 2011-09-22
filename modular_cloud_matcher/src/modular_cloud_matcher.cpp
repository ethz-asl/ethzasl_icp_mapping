#include "ros/ros.h"
#include "pointmatcher/PointMatcher.h"

#include "get_params_from_server.h"
#include "icp_chain_creation.h"
#include "cloud_conversion.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"

using namespace std;

class CloudMatcher
{
	ros::NodeHandle& n;
	
	PM::ICPSequence icp;
	
	const bool sendDeltaPoseMessage;
	const string fixedFrame;
	const string sensorFrame;
	const unsigned startupDropCount;
	unsigned dropCount;
	
	ros::Subscriber cloudSub;
	ros::Publisher pathPub;
	nav_msgs::Path path;
	tf::TransformBroadcaster br;
	ros::Publisher posePub;
	
public:
	CloudMatcher(ros::NodeHandle& n, const std::string &statFilePrefix, const bool sendDeltaPoseMessage);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsg);
};

CloudMatcher::CloudMatcher(ros::NodeHandle& n, const std::string &statFilePrefix, const bool sendDeltaPoseMessage):
	n(n),
	icp(statFilePrefix),
	sendDeltaPoseMessage(sendDeltaPoseMessage),
	fixedFrame(getParam<string>("fixedFrame",  "/world")),
	sensorFrame(getParam<string>("sensorFrame",  "/openni_rgb_optical_frame")),
	startupDropCount(getParam("startupDropCount", 0)),
	dropCount(0)
{
	const string cloudTopic(getParam<string>("cloudTopic", "/camera/rgb/points"));
	cloudSub = n.subscribe(cloudTopic, 1, &CloudMatcher::gotCloud, this);
	
	const string pathTopic(getParam<string>("path", "/tracker_path"));
	pathPub = n.advertise<nav_msgs::Path>(pathTopic, 1);
	
	path.header.frame_id = fixedFrame;
	
	populateParameters(icp);
	
	if (sendDeltaPoseMessage)
		posePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(getParam<string>("deltaPoseTopic", "/openni_delta_pose"), 3);
}

void CloudMatcher::gotCloud(const sensor_msgs::PointCloud2& cloudMsg)
{
	if (dropCount < startupDropCount)
	{
		++dropCount;
		return;
	}
	
	size_t goodCount(0);
	DP dp(rosMsgToPointMatcherCloud(cloudMsg, goodCount));
	
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		if (sendDeltaPoseMessage)
		{
			geometry_msgs::PoseWithCovarianceStamped pose;
			geometry_msgs::Point& position(pose.pose.pose.position);
			geometry_msgs::Quaternion& orientation(pose.pose.pose.orientation);
			
			pose.header.stamp = cloudMsg.header.stamp;
			
			position.x = numeric_limits<float>::quiet_NaN();
			position.y = numeric_limits<float>::quiet_NaN();
			position.z = numeric_limits<float>::quiet_NaN();
			orientation.x = numeric_limits<float>::quiet_NaN();
			orientation.y = numeric_limits<float>::quiet_NaN();
			orientation.z = numeric_limits<float>::quiet_NaN();
			orientation.w = numeric_limits<float>::quiet_NaN();
			
			posePub.publish(pose);
		}
		return;
	}
	
	const unsigned pointCount(cloudMsg.width * cloudMsg.height);
	ROS_INFO_STREAM("Got " << pointCount << " points (" << goodCount << " goods)");
	const double imageRatio = (double)goodCount / (double)pointCount;
	
	//TODO: put that as parameter, tricky to set...
	if (imageRatio < 0.5)
	{
		ROS_ERROR_STREAM("Partial cloud! Missing " << 100 - imageRatio*100.0 << "% of the cloud (received " << goodCount << ")");
		//return;
	}
	
	// call icp
	bool icpWasSuccess(true);
	try 
	{
		icp(dp);
		ROS_INFO_STREAM("match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl);
	}
	catch (PM::ConvergenceError error)
	{
		icpWasSuccess = false;
		ROS_WARN_STREAM("ICP failed to converge: " << error.what());
	}
	
	// broadcast transform
	if (sendDeltaPoseMessage)
	{
		const TP dTransform(icp.getDeltaTransform());
		const Eigen::eigen2_Quaternion<Scalar> dTquat(Matrix3(dTransform.block(0,0,3,3)));
		const Vector3 dTtr(dTransform.block(0,3,3,1));
		
		geometry_msgs::PoseWithCovarianceStamped pose;
		geometry_msgs::Point& position(pose.pose.pose.position);
		geometry_msgs::Quaternion& orientation(pose.pose.pose.orientation);
		
		pose.header.stamp = cloudMsg.header.stamp;
		
		if (icpWasSuccess)
		{
			position.x = dTtr(0);
			position.y = dTtr(1);
			position.z = dTtr(2);
			orientation.x = dTquat.x();
			orientation.y = dTquat.y();
			orientation.z = dTquat.z();
			orientation.w = dTquat.w();
		}
		else
		{
			ROS_WARN_STREAM("ICP failure in sendDeltaPose mode, resetting tracker");
			// we have a failure, so we are sure that we did not create a key frame, 
			// so dp is not affected. We can thus create a new keyframe
			icp.resetTracking(dp);
			position.x = numeric_limits<float>::quiet_NaN();
			position.y = numeric_limits<float>::quiet_NaN();
			position.z = numeric_limits<float>::quiet_NaN();
			orientation.x = numeric_limits<float>::quiet_NaN();
			orientation.y = numeric_limits<float>::quiet_NaN();
			orientation.z = numeric_limits<float>::quiet_NaN();
			orientation.w = numeric_limits<float>::quiet_NaN();
		}
		posePub.publish(pose);
	}
	
	// FIXME: should we continue publishing absolute pose as tf in sendDeltaPoseMessage mode?
	
	const TP globalTransform(icp.getTransform());
	const Eigen::eigen2_Quaternion<Scalar> quat(Matrix3(globalTransform.block(0,0,3,3)));
	
	tf::Quaternion tfQuat;
	tf::RotationEigenToTF(quat.cast<double>(), tfQuat);
	tf::Vector3 tfVect;
	tf::VectorEigenToTF(globalTransform.block(0,3,3,1).cast<double>(), tfVect);
	tf::Transform transform;
	transform.setRotation(tfQuat);
	transform.setOrigin(tfVect);

	if (icp.keyFrameCreatedAtLastCall())
	{
		ROS_WARN_STREAM("Keyframe created at " << icp.errorMinimizer->getWeightedPointUsedRatio());
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = sensorFrame;
		pose.pose.position.x = tfVect.x();
		pose.pose.position.y = tfVect.y();
		pose.pose.position.z = tfVect.z();
		pose.pose.orientation.x = tfQuat.x();
		pose.pose.orientation.y = tfQuat.y();
		pose.pose.orientation.z = tfQuat.z();
		pose.pose.orientation.w = tfQuat.w();
		path.poses.push_back(pose);
		pathPub.publish(path);
	}
	
	br.sendTransform(tf::StampedTransform(transform, cloudMsg.header.stamp, fixedFrame, sensorFrame));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_matcher_node");
	ros::NodeHandle n;
	bool sendDeltaPoseMessage(false);
	
	for (int i = 1; i < argc; ++i)
		if (strcmp(argv[i], "--senddeltapose") == 0)
			sendDeltaPoseMessage = true;
	
	CloudMatcher matcher(n, getParam<string>("statFilePrefix",  ""), sendDeltaPoseMessage);
	
	ros::spin();
	
	return 0;
}
