#include <fstream>

#include <boost/version.hpp>
#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
	#include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "map_msgs/GetPointMap.h"
#include "map_msgs/SaveMap.h"

using namespace std;
using namespace PointMatcherSupport;

class Mapper
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	
	ros::NodeHandle& n;
	ros::NodeHandle& pn;
	
	ros::Subscriber scanSub;
	ros::Subscriber cloudSub;
	ros::Publisher mapPub;
	ros::Publisher odomPub;
	ros::Publisher odomErrorPub;
	ros::ServiceServer getPointMapSrv;
	ros::ServiceServer saveMapSrv;
	ros::ServiceServer resetSrv;

	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapPreFilters;
	PM::DataPointsFilters mapPostFilters;
	unique_ptr<PM::Transformation> transformation;
	ros::Time mapCreationTime;
	PM::DataPoints *mapPointCloud;
	PM::ICPSequence icp;
	
	// multi-threading mapper
	#if BOOST_VERSION >= 104100
	typedef boost::packaged_task<PM::DataPoints*> MapBuildingTask;
	typedef boost::unique_future<PM::DataPoints*> MapBuildingFuture;
	boost::thread mapBuildingThread;
	MapBuildingTask mapBuildingTask;
	MapBuildingFuture mapBuildingFuture;
	bool mapBuildingInProgress;
	#endif // BOOST_VERSION >= 104100

	// Parameters
	int minReadingPointCount;
	int minMapPointCount;
	double minOverlap;
	double maxOverlapToMerge;
	double tfPublishPeriod;
	string odomFrame;
	string mapFrame;
	string vtkFinalMapName; //!< name of the final vtk map
	int inputQueueSize; 
	bool useConstMotionModel; 
	bool processingNewCloud; 

	PM::TransformationParameters TOdomToMap;
	boost::thread publishThread;
	boost::mutex publishLock;
	ros::Time publishStamp;
	
	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcaster;
	
public:
	Mapper(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Mapper();
	
protected:
	void gotScan(const sensor_msgs::LaserScan& scanMsgIn);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
	void processCloud(unique_ptr<DP> cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
	void processNewMapIfAvailable();
	void setMap(DP* newPointCloud);
	DP* updateMap(DP* newPointCloud, const PM::TransformationParameters Ticp, bool updateExisting);
	
	void publishLoop(double publishPeriod);
	void publishTransform();
	
	bool getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res);
	bool saveMap(map_msgs::SaveMap::Request &req, map_msgs::SaveMap::Response &res);
	bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

Mapper::Mapper(ros::NodeHandle& n, ros::NodeHandle& pn):
	n(n),
	pn(pn),
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	mapPointCloud(0),
	#if BOOST_VERSION >= 104100
	mapBuildingInProgress(false),
	#endif // BOOST_VERSION >= 104100
	minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
	minMapPointCount(getParam<int>("minMapPointCount", 500)),
	minOverlap(getParam<double>("minOverlap", 0.5)),
	maxOverlapToMerge(getParam<double>("maxOverlapToMerge", 0.9)),
	tfPublishPeriod(getParam<double>("tfPublishPeriod", 0.001)),
	odomFrame(getParam<string>("odom_frame", "odom")),
	mapFrame(getParam<string>("map_frame", "map")),
	vtkFinalMapName(getParam<string>("vtkFinalMapName", "finalMap.vtk")),
	inputQueueSize(getParam<int>("inputQueueSize", 10)),
	useConstMotionModel(getParam<bool>("useConstMotionModel", false)),
	processingNewCloud(false),
	publishStamp(ros::Time::now()),
  tfListener(ros::Duration(30))
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
	else
	{
		ROS_INFO_STREAM("No ICP config file given, using default");
		icp.setDefault();
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
	else
	{
		ROS_INFO_STREAM("No input filters config file given, not using these filters");
	}
	
	if (ros::param::get("~mapPreFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapPreFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map pre-filters config from YAML file " << configFileName);
		}
	}
	else
	{
		ROS_INFO_STREAM("No map pre-filters config file given, not using these filters");
	}
	
	if (ros::param::get("~mapPostFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapPostFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file " << configFileName);
		}
	}
	else
	{
		ROS_INFO_STREAM("No map post-filters config file given, not using these filters");
	}
	
	// topics and services initialization
	if (getParam<bool>("subscribe_scan", true))
		scanSub = n.subscribe("scan", inputQueueSize, &Mapper::gotScan, this);
	if (getParam<bool>("subscribe_cloud", true))
		cloudSub = n.subscribe("cloud_in", inputQueueSize, &Mapper::gotCloud, this);
	mapPub = n.advertise<sensor_msgs::PointCloud2>("point_map", 2, true);
	odomPub = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	odomErrorPub = n.advertise<nav_msgs::Odometry>("icp_error_odom", 50, true);
	getPointMapSrv = n.advertiseService("dynamic_point_map", &Mapper::getPointMap, this);
	saveMapSrv = pn.advertiseService("save_map", &Mapper::saveMap, this);
	resetSrv = pn.advertiseService("reset", &Mapper::reset, this);

	// refreshing tf transform thread
	publishThread = boost::thread(boost::bind(&Mapper::publishLoop, this, tfPublishPeriod));
}

Mapper::~Mapper()
{
	#if BOOST_VERSION >= 104100
	// wait for map-building thread
	if (mapBuildingInProgress)
	{
		mapBuildingFuture.wait();
		if (mapBuildingFuture.has_value())
			delete mapBuildingFuture.get();
	}
	#endif // BOOST_VERSION >= 104100
	// wait for publish thread
	publishThread.join();
	// save point cloud
	if (mapPointCloud)
	{
		mapPointCloud->save(vtkFinalMapName);
		delete mapPointCloud;
	}
}

void Mapper::gotScan(const sensor_msgs::LaserScan& scanMsgIn)
{
	const ros::Time endScanTime(scanMsgIn.header.stamp + ros::Duration(scanMsgIn.time_increment * (scanMsgIn.ranges.size() - 1)));
	unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scanMsgIn, &tfListener, odomFrame)));
	processCloud(move(cloud), scanMsgIn.header.frame_id, endScanTime, scanMsgIn.header.seq);
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
	processCloud(move(cloud), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
}

void Mapper::processCloud(unique_ptr<DP> newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{

	// FIXME confirm that modification. Intention: stop publishing tf with time now if a new point cloud is being processed.
	processingNewCloud = true;

	// if the future has completed, use the new map
	processNewMapIfAvailable();
	
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
	timer t;
	
	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

	ROS_INFO("Processing new point cloud");
	{
		timer t; // Print how long take the algo

		
		// Apply filters to incoming cloud, in scanner coordinates
		inputFilters.apply(*newPointCloud);
		
		ROS_INFO_STREAM("Input filters took " << t.elapsed() << " [s]");
	}
	
	string reason;
	// Initialize the transformation to identity if empty
 	if(!icp.hasMap())
 	{
		// we need to know the dimensionality of the point cloud to initialize properly
		TOdomToMap = PM::TransformationParameters::Identity(dimp1, dimp1);
	}

	// Fetch transformation from scanner to odom
	if (!tfListener.canTransform(scannerFrame,odomFrame,stamp,&reason))
	{
		ROS_ERROR_STREAM("Cannot lookup TOdomToScanner(" << odomFrame<< " to " << scannerFrame << "):\n" << reason);
		return;
	}

	const PM::TransformationParameters TOdomToScanner(
		PointMatcher_ros::eigenMatrixToDim<float>(
			PointMatcher_ros::transformListenerToEigenMatrix<float>(
			tfListener,
			scannerFrame,
			odomFrame,
			stamp
		), dimp1)
	);
	ROS_DEBUG_STREAM("TOdomToScanner(" << odomFrame<< " to " << scannerFrame << "):\n" << TOdomToScanner);
	ROS_DEBUG_STREAM("TOdomToMap(" << odomFrame<< " to " << mapFrame << "):\n" << TOdomToMap);
		
	const PM::TransformationParameters TscannerToMap = TOdomToMap * TOdomToScanner.inverse();
	ROS_DEBUG_STREAM("TscannerToMap (" << scannerFrame << " to " << mapFrame << "):\n" << TscannerToMap);
	
	// Ensure a minimum amount of point after filtering
	const int ptsCount = newPointCloud->features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts.");
		return;
	}

	// Initialize the map if empty
 	if(!icp.hasMap())
 	{
		ROS_INFO_STREAM("Creating an initial map");
		mapCreationTime = stamp;
		setMap(updateMap(newPointCloud.release(), TscannerToMap, false));
		// we must not delete newPointCloud because we just stored it in the mapPointCloud
		return;
	}
	
	// Check dimension
	if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1 << " while map is " << icp.getInternalMap().features.rows()-1);
		return;
	}
	
	try 
	{
		// Apply ICP
		PM::TransformationParameters Ticp;
		Ticp = icp(*newPointCloud, TscannerToMap);

		ROS_DEBUG_STREAM("Ticp:\n" << Ticp);
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		ROS_INFO_STREAM("Overlap: " << estimatedOverlap);
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			return;
		}
		
		// Compute tf
		publishStamp = stamp;
		publishLock.lock();
		TOdomToMap = Ticp * TOdomToScanner;
		processingNewCloud = false;
		publishLock.unlock();
		
		ROS_DEBUG_STREAM("TOdomToMap:\n" << TOdomToMap);

		// Publish tf
		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TOdomToMap, mapFrame, odomFrame, stamp));
		
		// Publish odometry
		if (odomPub.getNumSubscribers())
			odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, stamp));
	
		// Publish error on odometry
		if (odomErrorPub.getNumSubscribers())
			odomErrorPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TOdomToMap, mapFrame, stamp));

		// check if news points should be added to the map
		if (
			((estimatedOverlap < maxOverlapToMerge) || (icp.getInternalMap().features.cols() < minMapPointCount)) &&
			#if BOOST_VERSION >= 104100
			(!mapBuildingInProgress)
			#else // BOOST_VERSION >= 104100
			true
			#endif // BOOST_VERSION >= 104100
		)
		{
			// make sure we process the last available map
			mapCreationTime = stamp;
			#if BOOST_VERSION >= 104100
			ROS_INFO("Adding new points to the map in background");
			mapBuildingTask = MapBuildingTask(boost::bind(&Mapper::updateMap, this, newPointCloud.release(), Ticp, true));
			mapBuildingFuture = mapBuildingTask.get_future();
			mapBuildingThread = boost::thread(boost::move(boost::ref(mapBuildingTask)));
			mapBuildingInProgress = true;
			#else // BOOST_VERSION >= 104100
			ROS_INFO("Adding new points to the map");
			setMap(updateMap( newPointCloud.release(), Ticp, true));
			#endif // BOOST_VERSION >= 104100
		}
	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	ROS_INFO_STREAM("Total ICP took: " << t.elapsed() << " [s]");
}

void Mapper::processNewMapIfAvailable()
{
	#if BOOST_VERSION >= 104100
	if (mapBuildingInProgress && mapBuildingFuture.has_value())
	{
		ROS_INFO_STREAM("New map available");
		setMap(mapBuildingFuture.get());
		mapBuildingInProgress = false;
	}
	#endif // BOOST_VERSION >= 104100
}

void Mapper::setMap(DP* newPointCloud)
{
	// delete old map
	if (mapPointCloud)
		delete mapPointCloud;
	
	// set new map
	mapPointCloud = newPointCloud;
	icp.setMap(*mapPointCloud);
	
	// Publish map point cloud
	// FIXME this crash when used without descriptor
	if (mapPub.getNumSubscribers())
		mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, mapFrame, mapCreationTime));
}

Mapper::DP* Mapper::updateMap(DP* newPointCloud, const PM::TransformationParameters Ticp, bool updateExisting)
{
	timer t;
	
	// Correct new points using ICP result
	*newPointCloud = transformation->compute(*newPointCloud, Ticp); 
	
	// Preparation of cloud for inclusion in map
	mapPreFilters.apply(*newPointCloud);
	
	// Merge point clouds to map
	// FIXME: why map in newPointCloud instead of the inverse?
	if (updateExisting)
		newPointCloud->concatenate(*mapPointCloud);

	// Map maintenance
	// FIXME: why minMapPointCount confuse with the use of that parameter in the rest of the code
	if(newPointCloud->features.cols() > minMapPointCount)
		mapPostFilters.apply(*newPointCloud);
	
	ROS_INFO_STREAM("New map available (" << newPointCloud->features.cols() << " pts), update took " << t.elapsed() << " [s]");
	
	return newPointCloud;
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
	if(processingNewCloud == false && icp.hasMap())
	{
		publishLock.lock();
		// Note: we use now as timestamp to refresh the tf and avoid other buffer to be empty
		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TOdomToMap, mapFrame, odomFrame, ros::Time::now()));
		publishLock.unlock();
	}
}

bool Mapper::getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res)
{
	if (!mapPointCloud)
		return false;
	
	res.map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, mapFrame, ros::Time::now());
	return true;
}

bool Mapper::saveMap(map_msgs::SaveMap::Request &req, map_msgs::SaveMap::Response &res)
{
	if (!mapPointCloud)
		return false;
	
	try
	{
		mapPointCloud->save(req.filename.data);
	}
	catch (const std::runtime_error& e)
	{
		return false;
	}
	
	return true;
}

bool Mapper::reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	// note: no need for locking as we do ros::spin(), to update if we go for multi-threading
	TOdomToMap = PM::TransformationParameters::Identity(4,4);
	icp.clearMap();
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
