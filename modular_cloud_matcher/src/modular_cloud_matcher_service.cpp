#include "ros/ros.h"

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
	T v;
	if (ros::param::get(std::string("~")+name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}

#include "icp_chain_creation.h"

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "modular_cloud_matcher/MatchClouds.h"
#include "nav_msgs/Path.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h" 
#include "pcl/ros/conversions.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "pointmatcher/PointMatcher.h"

#if 0

using namespace std;

class CloudMatcher
{
	ros::NodeHandle& n;
	
	MSA::ICP icp;
	
	const string fixedFrame;
	const string sensorFrame;
	
	ros::ServiceServer service;
	
public:
	CloudMatcher(ros::NodeHandle& n);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsg);
};

CloudMatcher::CloudMatcher(ros::NodeHandle& n):
	n(n),
	fixedFrame(getParam<string>("fixedFrame",  "/world")),
	sensorFrame(getParam<string>("sensorFrame",  "/openni_rgb_optical_frame")),
	service(n.advertiseService(getParam<string>("serviceName","matchClouds"), &CloudMatcher::match, this))
{
	populateParameters(icp);
}

bool CloudMatcher::match(modular_cloud_matcher::MatchClouds::Request& req, modular_cloud_matcher::MatchClouds::Response& res)
{
	if (dropCount < startupDropCount)
	{
		++dropCount;
		return;
	}
	
	// create labels
	DP::Labels labels;
	labels.push_back(DP::Label("x", 1));
	labels.push_back(DP::Label("y", 1));
	labels.push_back(DP::Label("z", 1));
	labels.push_back(DP::Label("pad", 1));
	
	// create data points
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (cloudMsg, cloud);

	size_t goodCount(0);
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		if (!isnan(cloud.points[i].x))
			++goodCount;
	}
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
	
	DP dp(DP::Features(4, goodCount), labels);
	int dIndex(0);
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		if (!isnan(cloud.points[i].x))
		{
			dp.features.coeffRef(0, dIndex) = cloud.points[i].x;
			dp.features.coeffRef(1, dIndex) = cloud.points[i].y;
			dp.features.coeffRef(2, dIndex) = cloud.points[i].z;
			dp.features.coeffRef(3, dIndex) = 1;
			++dIndex;
		}
	}
	ROS_INFO_STREAM("Got " << cloud.points.size() << " points (" << goodCount << " goods)");
	
	const double imageRatio = (double)goodCount / (double)cloud.points.size();
	
	//TODO: put that as parameter, tricky to set...
	if (goodCount < 10000)
	{
		ROS_ERROR_STREAM("Partial image! Missing " << 100 - imageRatio*100.0 << "% of the image (received " << goodCount << ")");
		//return;
	}
	
	// call icp
	bool icpWasSuccess(true);
	try 
	{
		icp(dp);
		ROS_INFO_STREAM("match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl);
	}
	catch (MSA::ConvergenceError error)
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

#endif

int main(int argc, char **argv)
{
#if 0
	initParameters();
	
	ros::init(argc, argv, "cloud_matcher_node");
	ros::NodeHandle n;
	bool sendDeltaPoseMessage(false);
	
	for (int i = 1; i < argc; ++i)
		if (strcmp(argv[i], "--senddeltapose") == 0)
			sendDeltaPoseMessage = true;
	
	CloudMatcher matcher(n, getParam<string>("statFilePrefix",  ""), sendDeltaPoseMessage);
	
	ros::spin();
#endif
	return 0;
}

