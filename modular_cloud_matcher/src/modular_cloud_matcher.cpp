#include "icp_chain_creation.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "pointmatcher/PointMatcher.h"

using namespace std;

class CloudMatcher
{
	ros::NodeHandle& n;
	
	MSA::ICPSequence icp;
	
	const string fixedFrame;
	const string sensorFrame;
	const unsigned startupDropCount;
	unsigned dropCount;
	
	ros::Subscriber cloudSub;
	ros::Publisher pathPub;
	nav_msgs::Path path;
	tf::TransformBroadcaster br;
	
public:
	CloudMatcher(ros::NodeHandle& n, const std::string &statFilePrefix);
	void gotCloud(const sensor_msgs::PointCloud& cloudMsg);
};

CloudMatcher::CloudMatcher(ros::NodeHandle& n, const std::string &statFilePrefix):
	n(n),
	icp(3, statFilePrefix),
	fixedFrame(getParam<string>("fixedFrame",  "/world")),
	sensorFrame(getParam<string>("sensorFrame",  "/openni_rgb_optical_frame")),
	startupDropCount(getParam("startupDropCount", 0)),
	dropCount(0)
{
	const string cloudTopic(getParam<string>("cloudTopic", "/camera/depth/points"));
	cloudSub = n.subscribe(cloudTopic, 1, &CloudMatcher::gotCloud, this);
	
	const string pathTopic(getParam<string>("path", "/tracker_path"));
	pathPub = n.advertise<nav_msgs::Path>(pathTopic, 1);
	
	path.header.frame_id = fixedFrame;
	
	populateParameters(icp);
}

void CloudMatcher::gotCloud(const sensor_msgs::PointCloud& cloudMsg)
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
	size_t goodCount(0);
	for (size_t i = 0; i < cloudMsg.points.size(); ++i)
	{
		if (!isnan(cloudMsg.points[i].x))
			++goodCount;
	}
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	DP dp(DP::Features(4, goodCount), labels);
	int dIndex(0);
	for (size_t i = 0; i < cloudMsg.points.size(); ++i)
	{
		if (!isnan(cloudMsg.points[i].x))
		{
			dp.features.coeffRef(0, dIndex) = cloudMsg.points[i].x;
			dp.features.coeffRef(1, dIndex) = cloudMsg.points[i].y;
			dp.features.coeffRef(2, dIndex) = cloudMsg.points[i].z;
			dp.features.coeffRef(3, dIndex) = 1;
			++dIndex;
		}
	}
	ROS_INFO_STREAM("Got " << cloudMsg.points.size() << " points (" << goodCount << " goods)");
	
	const double imageRatio = (double)goodCount / (double)cloudMsg.points.size();
	
	//TODO: put that as parameter, tricky to set...
	if (goodCount < 10000)
	{
		ROS_ERROR_STREAM("Partial image! Missing " << 100 - imageRatio*100.0 << "% of the image (received " << goodCount << ")");
		return;
	}
	
	// call icp
	try 
	{
		icp(dp);
		ROS_INFO_STREAM("match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl);
	}
	catch (MSA::ConvergenceError error)
	{
		ROS_WARN_STREAM("ICP failed to converge: " << error.what());
	}
	
	// broadcast transform
	const TP globalTransform(icp.getTransform());
	const Eigen::Quaternion<Scalar> quat(Matrix3(globalTransform.block(0,0,3,3)));
	
	tf::Quaternion tfQuat;
	tf::RotationEigenToTF(Eigen::Quaterniond(quat), tfQuat);
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
	initParameters();
	
	ros::init(argc, argv, "cloud_matcher_node");
	ros::NodeHandle n;
	
	CloudMatcher matcher(n, getParam<string>("statFilePrefix",  ""));
	
	ros::spin();
	
	return 0;
}
