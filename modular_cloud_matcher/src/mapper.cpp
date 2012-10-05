#include <fstream>

#include <boost/thread.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/aliases.h"
#include "pointmatcher_ros/ros_logger.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "mapping_msgs/GetPointMap.h"
#include "mapping_msgs/SaveMap.h"

using namespace std;
using namespace PointMatcherSupport;

class Mapper
{
	ros::NodeHandle& n;
	ros::NodeHandle& pn;
	
	ros::Subscriber scanSub;
	ros::Subscriber cloudSub;
	ros::Publisher mapPub;
	ros::Publisher odomPub;
	ros::ServiceServer getPointMapSrv;
	ros::ServiceServer republishMapSrv;
	ros::ServiceServer saveMapSrv;
	ros::ServiceServer resetSrv;

	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapFilters;
	shared_ptr<PM::Transformation> transformation;
	PM::DataPoints mapPointCloud;
	sensor_msgs::PointCloud2 mapMsg;
	PM::ICP icp;

	// Parameters
	int minReadingPointCount;
	int minMapPointCount;
	double minOverlap;
	double maxOverlapToMerge;
	double tfPublishPeriod;
	string odomFrame;
	string mapFrame;
	string vtkGlobalMapPrefix; //!< if empty, no vtk dump at every scan
	string vtkFinalMapName; //!< name of the final vtk map

	PM::TransformationParameters Ticp;
	boost::thread* publishThread;
	boost::mutex publishLock;
	
	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcaster;
	
public:
	Mapper(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Mapper();
	
protected:
	void gotScan(const sensor_msgs::LaserScan& scanMsgIn);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
	void processCloud(DP& cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);

	void globalMapMaintenance();
	void publishLoop(double publishPeriod);
	void publishTransform();
	
	bool getPointMap(mapping_msgs::GetPointMap::Request &req, mapping_msgs::GetPointMap::Response &res);
	bool republishMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool saveMap(mapping_msgs::SaveMap::Request &req, mapping_msgs::SaveMap::Response &res);
	bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

Mapper::Mapper(ros::NodeHandle& n, ros::NodeHandle& pn):
	n(n),
	pn(pn),
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
	minMapPointCount(getParam<int>("minMapPointCount", 500)),
	minOverlap(getParam<double>("minOverlap", 0.5)),
	maxOverlapToMerge(getParam<double>("maxOverlapToMerge", 0.9)),
	tfPublishPeriod(getParam<double>("tfPublishPeriod", 0.1)),
	odomFrame(getParam<string>("odom_frame", "odom")),
	mapFrame(getParam<string>("map_frame", "map")),
	vtkGlobalMapPrefix(getParam<string>("vtkGlobalMapPrefix", "")),
	vtkFinalMapName(getParam<string>("vtkFinalMapName", "uniformMap")),
	Ticp(PM::TransformationParameters::Identity(4,4)),
	publishThread(0)
{
	// set logger
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);

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
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);
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
	
	// topics and services initialization
	scanSub = n.subscribe("scan", 2, &Mapper::gotScan, this);
	cloudSub = n.subscribe("cloud_in", 2, &Mapper::gotCloud, this);
	mapPub = n.advertise<sensor_msgs::PointCloud2>("point_map", 2);
	odomPub = n.advertise<nav_msgs::Odometry>("icp_odom", 50);
	getPointMapSrv = n.advertiseService("dynamic_point_map", &Mapper::getPointMap, this);
	republishMapSrv = pn.advertiseService("republish_map", &Mapper::republishMap, this);
	saveMapSrv = pn.advertiseService("save_map", &Mapper::saveMap, this);
	resetSrv = pn.advertiseService("reset", &Mapper::reset, this);

	// initial transform
	publishTransform();
	publishThread = new boost::thread(boost::bind(&Mapper::publishLoop, this, tfPublishPeriod));
}

Mapper::~Mapper()
{
	PM::saveVTK(mapPointCloud, vtkFinalMapName);
	if (publishThread)
	{
		publishThread->join();
		delete publishThread;
	}
}

void Mapper::gotScan(const sensor_msgs::LaserScan& scanMsgIn)
{
	const ros::Time endScanTime(scanMsgIn.header.stamp + ros::Duration(scanMsgIn.time_increment * (scanMsgIn.ranges.size() - 1)));
	DP cloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scanMsgIn, &tfListener, odomFrame));
	processCloud(cloud, scanMsgIn.header.frame_id, endScanTime, scanMsgIn.header.seq);
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	DP cloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));
	processCloud(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
}

void Mapper::processCloud(DP& newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
	timer t;
	
	// Convert point cloud
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
	const int dimp1(newPointCloud.features.rows());
	
	// Fetch initial guess and transform cloud with it
	try
	{
		const PM::TransformationParameters TscannerToOdom(
			PointMatcher_ros::eigenMatrixToDim<float>(
				PointMatcher_ros::transformListenerToEigenMatrix<float>(
					tfListener, 
					odomFrame,
					scannerFrame,
					stamp 
				), dimp1
			)
		);
		newPointCloud = transformation->compute(newPointCloud, TscannerToOdom);
		ROS_INFO_STREAM("TscannerToOdom:\n" << TscannerToOdom);
	}
	catch (const tf::ExtrapolationException& e)
	{
		ROS_ERROR_STREAM("TF extrapolation exception: " << e.what());
		return;
	}
	
	PM::TransformationParameters Tinit(PointMatcher_ros::eigenMatrixToDim<float>(Ticp, dimp1));
	if (tfListener.canTransform(odomFrame,mapFrame,stamp))
	{
		Tinit = PointMatcher_ros::eigenMatrixToDim<float>(
			PointMatcher_ros::transformListenerToEigenMatrix<float>(
			tfListener,
			mapFrame,
			odomFrame,
			stamp
		), dimp1);
	}
	else
		ROS_WARN("Cannot lookup Tinit, using last Ticp");
	ROS_INFO_STREAM("Tinit:\n" << Tinit);
	
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
		//cerr << "map point cloud:\n" << mapPointCloud.features.leftCols(10) << endl;
		mapMsg = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, mapFrame, stamp);
		if (mapPub.getNumSubscribers())
			mapPub.publish(mapMsg);
		return;
	}
	
	// check dimension
	if (newPointCloud.features.rows() != mapPointCloud.features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud.features.rows()-1 << " while map is " << mapPointCloud.features.rows()-1);
		return;
	}
	
	// Apply ICP
	try 
	{
		// TODO: have a way to pass store matcher's kdtree accross calls, prevent call of matcher::init()
		PM::TransformationParameters T = icp(newPointCloud, mapPointCloud, Tinit);
		//T = PM::TransformationParameters::Identity(newPointCloud.features.rows(), newPointCloud.features.rows());
		ROS_INFO_STREAM("Ticp:\n" << T);

		//transformation->compute(mapPointCloud, T.inverse());
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		ROS_INFO_STREAM("Overlap: " << estimatedOverlap);
		if (estimatedOverlap < minOverlap)
		{
			publishLock.lock();
			Ticp = Tinit;
			publishLock.unlock();
			ROS_ERROR_STREAM("Estimated overlap too small, move back!");
			return;
		}
		
		publishLock.lock();
		Ticp = T;
		publishLock.unlock();
		
		// Publish odometry message
		if (odomPub.getNumSubscribers())
			odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(T, mapFrame, stamp));
		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T, mapFrame, odomFrame, stamp));
		
		// check if news points should be added to the map
		if ((estimatedOverlap < maxOverlapToMerge) || (mapPointCloud.features.cols() < minMapPointCount))
		{
			ROS_INFO("Adding new points to the map");
			
			// Merge point clouds to map
			newPointCloud = transformation->compute(newPointCloud, Ticp); 
			mapPointCloud.concatenate(newPointCloud);

			// Map maintenance
			mapFilters.apply(mapPointCloud);
			
			// Publish map point cloud
			mapMsg = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, mapFrame, stamp);
			if (mapPub.getNumSubscribers())
				mapPub.publish(mapMsg);
		}
	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	ROS_INFO_STREAM("Mapping total time (ICP+maintenance): " << t.elapsed() << " sec");

	if(!vtkGlobalMapPrefix.empty())
	{
		stringstream nameStream;
		nameStream << vtkGlobalMapPrefix << seq;
		PM::saveVTK(mapPointCloud, nameStream.str());
	}
}

void Mapper::publishLoop(double publishPeriod)
{
	if(publishPeriod == 0)
		return;
	ros::Rate r(1.0 / publishPeriod);
	while(ros::ok())
	{
		publishTransform();
		r.sleep();
	}
}

void Mapper::publishTransform()
{
	publishLock.lock();
	tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticp, mapFrame, odomFrame, ros::Time::now()));
	publishLock.unlock();
}

bool Mapper::getPointMap(mapping_msgs::GetPointMap::Request &req, mapping_msgs::GetPointMap::Response &res)
{
	// note: no need for locking as we do ros::spin(), to update if we go for multi-threading
	res.map = mapMsg;
	return true;
}

bool Mapper::saveMap(mapping_msgs::SaveMap::Request &req, mapping_msgs::SaveMap::Response &res)
{
	try
	{
		PM::saveAnyFormat(mapPointCloud, req.filename.data);
	}
	catch (const std::runtime_error& e)
	{
		return false;
	}
	
	return true;
}

bool Mapper::republishMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	if (mapPub.getNumSubscribers())
		mapPub.publish(mapMsg);
	return true;
}

bool Mapper::reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	// note: no need for locking as we do ros::spin(), to update if we go for multi-threading
	Ticp = PM::TransformationParameters::Identity(4,4);
	mapPointCloud = DP();
	return true;
}

// Main function supporting the Mapper class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	Mapper mapper(n, pn);
	ros::spin();
	
	return 0;
}
