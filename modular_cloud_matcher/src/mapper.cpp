#include <fstream>

#include <boost/thread.hpp>

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
#include "tf/transform_broadcaster.h"


using namespace std;
using namespace PointMatcherSupport;

class Mapper
{
	ros::NodeHandle& n;
	
	ros::Subscriber scanSub;
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
	Mapper(ros::NodeHandle& n);
	~Mapper();
	
protected:
	void gotScan(const sensor_msgs::LaserScan& scanMsgIn);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
	void processCloud(DP& cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);

	void globalMapMaintenance();
	void publishLoop(double publishPeriod);
	void publishTransform();
};

Mapper::Mapper(ros::NodeHandle& n):
	n(n),
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
	
	// topics initialization
	if (hasParam("scanTopic"))
	{
		scanSub = n.subscribe(
			getParam<string>("scanTopic", "/laser_scan"), 2, &Mapper::gotScan, this
		);
	}
	if (hasParam("cloudTopic"))
	{
		cloudSub = n.subscribe(
			getParam<string>("cloudTopic", "/point_cloud"), 2, &Mapper::gotCloud, this
		);
	}
	mapPub = n.advertise<sensor_msgs::PointCloud2>(
		getParam<string>("pointMapTopic", "/point_map"), 1
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
		if (mapPub.getNumSubscribers())
			mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, mapFrame, stamp));
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
		//odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(T, mapFrame, stamp));
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
			if (mapPub.getNumSubscribers())
				mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, mapFrame, stamp));
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


// Main function supporting the Mapper class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper");
	ros::NodeHandle n;
	Mapper mapper(n);
	ros::spin();
	
	return 0;
}
