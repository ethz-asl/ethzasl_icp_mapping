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

	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapFilters;
	shared_ptr<PM::Transformation> transformation;
	PM::DataPoints mapPointCloud;
	PM::ICP icp;

	// Parameters
	int minReadingPointCount;
	double minOverlap;
	string mapFrame;
	string vtkGlobalMapPrefix; //!< if empty, no vtk dump at every scan
	string vtkFinalMapName; //!< name of the final vtk map

	tf::TransformListener tf_listener;

public:
	Mapper(ros::NodeHandle& n);
	~Mapper();
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsg);

private:
	void globalMapMaintenance();
};

Mapper::Mapper(ros::NodeHandle& n):
	n(n),
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
	minOverlap(getParam<double>("minOverlap", 0.5)),
	mapFrame(getParam<string>("mapFrameId", "/map")),
	vtkGlobalMapPrefix(getParam<string>("vtkGlobalMapPrefix", "")),
	vtkFinalMapName(getParam<string>("vtkFinalMapName", "uniformMap"))
{
	// topics initialization
	cloudSub = n.subscribe(
		getParam<string>("cloudTopic", "/static_point_cloud"), 2, &Mapper::gotCloud, this
	);
	mapPub = n.advertise<sensor_msgs::PointCloud2>(
		getParam<string>("mapTopic", "/map3D"), 1
	);
	odomPub = n.advertise<nav_msgs::Odometry>(
		getParam<string>("odomTopic", "/icp_odom"), 50
	);

	// load configs
	string configFileName;
	if (ros::param::get("~icpConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icp.loadFromYaml(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load ICP config from YAML file " << configFileName);
			icp.setDefault();
		}
	}
	if (ros::param::get("~inputFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			inputFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load input filters config from YAML file " << configFileName);
		}
	}
	if (ros::param::get("~mapFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map filters config from YAML file " << configFileName);
		}
	}

	//setLogger(pm.LoggerRegistrar.create("FileLogger"));
}


void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
	timer t;
	
	// Convert point cloud
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
	
	// Apply filters to incoming cloud
	inputFilters.apply(newPointCloud);
	
	// Fetch initial guess and transform cloud with it
	const PM::TransformationParameters Tinit(
		PointMatcher_ros::transformListenerToEigenMatrix<float>(
			tf_listener, 
			cloudMsgIn.header.frame_id, 
			mapFrame,
			cloudMsgIn.header.stamp 
		)
	);
	newPointCloud = transformation->compute(newPointCloud, Tinit);

	// Ensure a minimum amount of point after filtering
	const int ptsCount = newPointCloud.features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts.");
		return;
	}

	// Initialize the map if empty
	if(mapPointCloud.features.rows() == 0)
	{
		mapPointCloud = newPointCloud;
		return;
	}
	
	// Apply ICP
	try 
	{
		const PM::TransformationParameters T = icp(newPointCloud, mapPointCloud);
		
		// FIXME: why this direction, why moving the map always?
		transformation->compute(mapPointCloud, T.inverse());
		
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
		// FIXME: why not TF
	}
	catch (PM::ConvergenceError error)
	{
		ROS_WARN_STREAM("ICP failed to converge: " << error.what());
	}

	// Merge point clouds to map
	mapPointCloud.concatenate(newPointCloud);

	// Map maintenance
	mapFilters.apply(mapPointCloud);
	ROS_INFO_STREAM("Mapping total time (ICP+maintenance): " << t.elapsed() << " sec");

	// Publish map point cloud
	mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, mapFrame, cloudMsgIn.header.stamp));

	if(!vtkGlobalMapPrefix.empty())
	{
		stringstream nameStream;
		nameStream << vtkGlobalMapPrefix << cloudMsgIn.header.seq;
		PM::saveVTK(mapPointCloud, nameStream.str());
	}
}

Mapper::~Mapper()
{
	PM::saveVTK(mapPointCloud, vtkFinalMapName);
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
