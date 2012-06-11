#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include "aliases.h"
#include "get_params_from_server.h"

#include "ros_logger.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"

using namespace std;
using namespace PointMatcherSupport;

class Mapper
{
	ros::NodeHandle& n;
	
	ros::Subscriber cloudSub;
	ros::Publisher mapPub;
	ros::Publisher odomPub;

	PM pm;
	PM::DataPoints mapPointCloud;
	PM::ICP icp;
	PM::Transformations transformations; 

	// Parameters
	int maxMapPointCount;
	int minReadingPointCount;
	int maxReadingPointCount;
	double minOverlap;
	string mapFrame;
	bool dumpVTKGlobalMap;

	tf::TransformListener tf_listener;

public:
	Mapper(ros::NodeHandle& n);
	~Mapper();
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsg);
private:
	PM::DataPoints filterScannerPtsCloud(const PM::DataPoints pointCloud);
	void globalMapMaintenance();
};

Mapper::Mapper(ros::NodeHandle& n):
	n(n)
{

	// ROS initialization
	const string cloudTopic(getParam<string>("cloudTopic", "/static_point_cloud"));
	cloudSub = n.subscribe(cloudTopic, 2, &Mapper::gotCloud, this);
	
	const string mapTopic(getParam<string>("mapTopic", "/map3D"));
	mapPub = n.advertise<sensor_msgs::PointCloud2>(mapTopic, 1);
		
	odomPub = n.advertise<nav_msgs::Odometry>(getParam<string>("odomTopic", "/icp_odom"), 50);

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

	//setLogger(pm.LoggerRegistrar.create("FileLogger"));
	
	// Prepare transformation chain for maps
	PM::Transformation* transformPoints;
	transformPoints = pm.TransformationRegistrar.create("TransformFeatures");
	PM::Transformation* transformNormals;
	transformNormals = pm.TransformationRegistrar.create("TransformNormals");
	
	transformations.push_back(transformPoints);
	transformations.push_back(transformNormals);


	// Parameters for 3D map
	maxMapPointCount = getParam<int>("maxMapPointCount", 300000);
	minReadingPointCount = getParam<int>("minReadingPointCount", 2000);
	maxReadingPointCount = getParam<int>("maxReadingPointCount", 10000);
	minOverlap = getParam<double>("minOverlap", 0.5);
	mapFrame = getParam<string>("mapFrameId", "/map");
	dumpVTKGlobalMap = getParam<bool>("dumpVTKGlobalMap", false);
}


void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
	timer t;
	t.restart();

	// Fetch initial guess for transformation
	const PM::TransformationParameters Tinit(
		PointMatcher_ros::transformListenerToEigenMatrix<float>(
			tf_listener, 
			cloudMsgIn.header.frame_id, 
			mapFrame,
			cloudMsgIn.header.stamp 
		)
	);

	DP newPointCloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));
	const size_t goodCount(newPointCloud.features.cols());
	
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	else
	{
		ROS_INFO("Processing new point cloud");
	}
		
	//bool icpWasSuccess = true;
	
	newPointCloud = filterScannerPtsCloud(newPointCloud);
	
	// Transform point with initial guess
	transformations.apply(newPointCloud, Tinit);

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
		PM::TransformationParameters T = icp(newPointCloud, mapPointCloud);
		
		transformations.apply(mapPointCloud, T.inverse());

		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small (" << estimatedOverlap << "), move back");	
			return;
		}

		// Publish odometry message
		odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(T, mapFrame, cloudMsgIn.header.stamp));
		
		ROS_INFO_STREAM("**** Adding new points to the map with " << estimatedOverlap << " overlap");

	
		// Merge point clouds to map			
		mapPointCloud.concatenate(newPointCloud);
	
		// Map maintenance
		globalMapMaintenance();

		ROS_INFO_STREAM("Mapping total time (ICP+maintenance): " << t.elapsed() << " sec");

		//Publish map point cloud
		mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, mapFrame, cloudMsgIn.header.stamp));

		if(dumpVTKGlobalMap)
		{
			stringstream nameStream;
			nameStream << "nifti_map_" << cloudMsgIn.header.seq;
			PM::saveVTK(mapPointCloud, nameStream.str());
		}
		
	}
	catch (PM::ConvergenceError error)
	{
		//icpWasSuccess = false;
		ROS_WARN_STREAM("ICP failed to converge: " << error.what());
	}

	
}

//Warning! need to have pointCloud in laser frame or very close
PM::DataPoints Mapper::filterScannerPtsCloud(const PM::DataPoints pointCloud)
{
	// Create a copy of the points
	PM::DataPoints newPointCloud = pointCloud;

	// Construct sensor noise
	PM::DataPointsFilter* sensorNoiseFilter;
	sensorNoiseFilter= pm.DataPointsFilterRegistrar.create(
		"SimpleSensorNoiseDataPointsFilter");
	newPointCloud = sensorNoiseFilter->filter(newPointCloud);

	// Build filter to compute normals and down sample
	const double probToKeep = maxReadingPointCount/(double)pointCloud.features.cols();
	if(probToKeep < 1.0)
	{
		PM::DataPointsFilter* normalFilter;
		normalFilter = pm.DataPointsFilterRegistrar.create(
			"SamplingSurfaceNormalDataPointsFilter", PM::Parameters({
				{"binSize", "20"},
				{"epsilon", "5"}, 
				{"ratio", toParam(probToKeep)}, 
				{"keepNormals", "1"}, 
				{"keepDensities", "1"}
				}));

		newPointCloud = normalFilter->filter(newPointCloud);
	}

	// Point normal vector toward laser
	PM::DataPointsFilter* orienteNormalFilter;
	orienteNormalFilter= pm.DataPointsFilterRegistrar.create(
		"OrientNormalsDataPointsFilter");
	newPointCloud = orienteNormalFilter->filter(newPointCloud);

	// Remove shadow points (mixed pixel)
	PM::DataPointsFilter* shadowFilter;
	shadowFilter = pm.DataPointsFilterRegistrar.create(
		"ShadowDataPointsFilter");
	//newPointCloud = shadowFilter->filter(newPointCloud);
	

	return newPointCloud;
}

void Mapper::globalMapMaintenance()
{
	// Generic holder for parameters
	PM::Parameters params;

	// Uniformize density
	PM::DataPointsFilter* uniformSubsample;
	params = PM::Parameters({{"aggressivity", toParam(0.65)}});
	uniformSubsample = pm.DataPointsFilterRegistrar.create("UniformizeDensityDataPointsFilter", params);
	
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
