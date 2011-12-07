#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"

#include "aliases.h"
#include "get_params_from_server.h"
#include "cloud_conversion.h"
#include "ros_logger.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include <pcl_ros/transforms.h>

using namespace std;
using namespace PointMatcherSupport;

class Mapper
{
	ros::NodeHandle& n;
	
	ros::Subscriber cloudSub;
	ros::Publisher mapPub;
	ros::Publisher odomPub;

	PM::DataPoints mapPointCloud;
	PM::ICP icp;
	PM pm;
	int totalPointCount;
	string mapFrame;

	tf::TransformListener tf_listener;

public:
	Mapper(ros::NodeHandle& n);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsg);
};

Mapper::Mapper(ros::NodeHandle& n):
	n(n)
{

	// ROS initialization
	const string cloudTopic(getParam<string>("cloudTopic", "/nifti_point_cloud"));
	cloudSub = n.subscribe(cloudTopic, 1, &Mapper::gotCloud, this);
	
	const string mapTopic(getParam<string>("mapTopic", "/map3D"));
	mapPub = n.advertise<sensor_msgs::PointCloud2>(mapTopic, 1);
	
	const string odomTopic(getParam<string>("odomTopic", "/odomICP"));
	odomPub = n.advertise<nav_msgs::Odometry>(odomTopic, 1);
		
	// load config
	string configFileName;
	if (ros::param::get("~config", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icp.loadFromYaml(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load config from YAML file " << configFileName);
			icp.setDefault();
		}
	}

	// Parameters for 3D map
	totalPointCount = getParam<int>("totalPointCount", 100000);
	mapFrame= getParam<string>("mapFrameId", "/map");
}


void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{

	sensor_msgs::PointCloud2 cloudMsg;

	pcl_ros::transformPointCloud(mapFrame, cloudMsgIn, cloudMsg, tf_listener);

	size_t goodCount(0);
	DP newPointCloud(rosMsgToPointMatcherCloud(cloudMsg, goodCount));
	
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
		
	PM::TransformationParameters T = PM::TransformationParameters::Identity(4,4);
	bool icpWasSuccess = true;
	
	// Keep the map in memory
	if(mapPointCloud.features.rows() == 0)
	{
		// Initialize the map
		mapPointCloud = newPointCloud;
	}
	
	// Apply ICP
	try 
	{
		T = icp(newPointCloud, mapPointCloud);
		cout << T.inverse() << endl;
		mapPointCloud.features = T.inverse() * mapPointCloud.features;
	
		const double estimatedOverlap = icp.errorMinimizer->getWeightedPointUsedRatio();
		//if (estimatedOverlap < 0.85)
		{
			ROS_INFO_STREAM("Adding new points to the map with " << estimatedOverlap << " overlap");
	
			// Create point cloud filters
			PM::DataPointsFilter* uniformSubsample;
			const PM::Parameters params({{"aggressivity", toParam(0.5)}});
			uniformSubsample = pm.DataPointsFilterRegistrar.create("UniformizeDensityDataPointsFilter", params);
			
					
			// Controle the size of the point cloud
			mapPointCloud = concatenatClouds(mapPointCloud, newPointCloud);
			mapPointCloud = uniformSubsample->filter(mapPointCloud);

			const double probToKeep = totalPointCount/mapPointCloud.features.cols();
			if(probToKeep < 1)
			{
				PM::DataPointsFilter* randSubsample;
				const PM::Parameters params({{"prob", toParam(probToKeep)}});
				randSubsample = pm.DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", params);
				mapPointCloud = randSubsample->filter(mapPointCloud);
			}

			mapPub.publish(pointMatcherCloudToRosMsg(mapPointCloud, mapFrame));
		}
	}
	catch (PM::ConvergenceError error)
	{
		icpWasSuccess = false;
		ROS_WARN_STREAM("ICP failed to converge: " << error.what());
	}

	if(icpWasSuccess)
	{
		tf::Quaternion tfQuat;
#if ROS_VERSION_MINIMUM(1, 6, 0)
		// electric and later
		const Eigen::Quaternion<Scalar> quat(Matrix3(T.block(0,0,3,3)));
		tf::RotationEigenToTF(quat.cast<double>(), tfQuat);
#else // ROS_VERSION_MINIMUM(1, 6, 0)
		// diamondback and before
		const Eigen::eigen2_Quaternion<Scalar> quat(Matrix3(T.block(0,0,3,3)));
		tf::RotationEigenToTF(quat.cast<double>(), tfQuat);
#endif // ROS_VERSION_MINIMUM(1, 6, 0)
		tf::Vector3 tfVect;
		tf::VectorEigenToTF(T.block(0,3,3,1).cast<double>(), tfVect);

		
		nav_msgs::Odometry odom;
		odom.header.frame_id = mapFrame;
		//TODO: should it be point cloud time?
		odom.header.stamp = ros::Time::now(); 
		
		geometry_msgs::PoseWithCovariance pose;
		
		pose.pose.position.x = tfVect.x();
		pose.pose.position.y = tfVect.y();
		pose.pose.position.z = tfVect.z();
		pose.pose.orientation.x = tfQuat.x();
		pose.pose.orientation.y = tfQuat.y();
		pose.pose.orientation.z = tfQuat.z();
		pose.pose.orientation.w = tfQuat.w();

		boost::array<double, 36> cov = {{1, 0, 0, 0, 0, 0,
												0, 1, 0, 0, 0, 0,
												0, 0, 1, 0, 0, 0,
												0, 0, 0, 1, 0, 0,
												0, 0, 0, 0, 1, 0,
												0, 0, 0, 0, 0, 1}};
		
		pose.covariance.swap(cov);
	
		odom.pose = pose;
		odomPub.publish(odom);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper_node");
	ros::NodeHandle n;
	
	Mapper mapper(n);
	
	ros::spin();
	
	return 0;
}
