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
	//setLogger(pm.LoggerRegistrar.create("FileLogger"));
	int maxMapPointCount;
	int minReadingPointCount;
	double minOverlap;
	string mapFrame;

	tf::TransformListener tf_listener;

public:
	Mapper(ros::NodeHandle& n);
	~Mapper();
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsg);
private:
	PM::DataPoints filterScannerPtsCloud(const PM::DataPoints pointCloud);
};

Mapper::Mapper(ros::NodeHandle& n):
	n(n)
{

	// ROS initialization
	const string cloudTopic(getParam<string>("cloudTopic", "/static_point_cloud"));
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
	maxMapPointCount = getParam<int>("maxMapPointCount", 300000);
	minReadingPointCount = getParam<int>("minReadingPointCount", 5000);
	minOverlap = getParam<double>("minOverlap", 0.5);
	mapFrame= getParam<string>("mapFrameId", "/map");
}


void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	PM::Parameters params;
	PM::TransformationParameters T = PM::TransformationParameters::Identity(4,4);
	
	sensor_msgs::PointCloud2 cloudMsg;

	pcl_ros::transformPointCloud(mapFrame, cloudMsgIn, cloudMsg, tf_listener);

	size_t goodCount(0);
	DP newPointCloud(rosMsgToPointMatcherCloud(cloudMsg, goodCount));
	
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	else
	{
		ROS_INFO("Processing new point cloud");
	}
		
	bool icpWasSuccess = true;
	
	newPointCloud = filterScannerPtsCloud(newPointCloud);
	
	// Ensure a minimum amount of point after filtering
	const int ptsCount = newPointCloud.features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: " << ptsCount << "pts.");
		return;
	}

	// Keep the map in memory
	if(mapPointCloud.features.rows() == 0)
	{
		// Initialize the map
		mapPointCloud = newPointCloud;
		return;
	}
	
	// Apply ICP
	try 
	{
		T = icp(newPointCloud, mapPointCloud);
		mapPointCloud.features = T.inverse() * mapPointCloud.features;

		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getWeightedPointUsedRatio();
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR("Estimated overlap too small, move back");	
			return;
		}

		ROS_INFO_STREAM("**** Adding new points to the map with " << estimatedOverlap << " overlap");

		// Create point cloud filters
		PM::DataPointsFilter* uniformSubsample;
		params = PM::Parameters({{"aggressivity", toParam(0.5)}});
		uniformSubsample = pm.DataPointsFilterRegistrar.create("UniformizeDensityDataPointsFilter", params);
		
		// Merge point clouds to map			
		mapPointCloud.concatenate(newPointCloud);
		mapPointCloud = uniformSubsample->filter(mapPointCloud);

		// Controle the size of the point cloud
		const double probToKeep = maxMapPointCount/(double)mapPointCloud.features.cols();
		if(probToKeep < 1.0)
		{
			PM::DataPointsFilter* randSubsample;
			params = PM::Parameters({{"prob", toParam(probToKeep)}});
			randSubsample = pm.DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", params);
			mapPointCloud = randSubsample->filter(mapPointCloud);
		}

		//Publish map point cloud
		stringstream nameStream;
		nameStream << "nifti_map_" << cloudMsg.header.seq;
		PM::saveVTK(mapPointCloud, nameStream.str());
		mapPub.publish(pointMatcherCloudToRosMsg(mapPointCloud, mapFrame));
		
	}
	catch (PM::ConvergenceError error)
	{
		icpWasSuccess = false;
		ROS_WARN_STREAM("ICP failed to converge: " << error.what());
	}

	
}

//Warning! need to have pointCloud in laser frame or very close
PM::DataPoints Mapper::filterScannerPtsCloud(const PM::DataPoints pointCloud)
{
	// Build filter to remove shadow points and down sample
	PM::DataPointsFilter* normalFilter;
	normalFilter = pm.DataPointsFilterRegistrar.create(
		"SamplingSurfaceNormalDataPointsFilter", PM::Parameters({
			{"binSize", "20"},
			{"epsilon", "5"}, 
			{"ratio", "0.25"}, 
			{"keepNormals", "1"}, 
			{"keepDensities", "1"}
			}));

	PM::DataPointsFilter* orienteNormalFilter;
	orienteNormalFilter= pm.DataPointsFilterRegistrar.create(
		"OrientNormalsDataPointsFilter");

	PM::DataPointsFilter* shadowFilter;
	shadowFilter = pm.DataPointsFilterRegistrar.create(
		"ShadowDataPointsFilter");
	
	// Apply filter in laser coordinates 
	PM::DataPoints newPointCloud = pointCloud;
	newPointCloud = normalFilter->filter(newPointCloud);
	newPointCloud = orienteNormalFilter->filter(newPointCloud);
	newPointCloud = shadowFilter->filter(newPointCloud);

	return newPointCloud;
}

Mapper::~Mapper()
{
	PM::saveVTK(mapPointCloud, "uniformMap");	
}

// Main function supporting the Mapper class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper_node");
	ros::NodeHandle n;
	Mapper mapper(n);
	ros::spin();
	
	return 0;
}
