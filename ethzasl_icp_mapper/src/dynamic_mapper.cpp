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

#include "nabo/nabo.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

// Services
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "std_srvs/Empty.h"
#include "map_msgs/GetPointMap.h"
#include "map_msgs/SaveMap.h"
#include "ethzasl_icp_mapper/LoadMap.h"
#include "ethzasl_icp_mapper/CorrectPose.h"
#include "ethzasl_icp_mapper/SetMode.h"
#include "ethzasl_icp_mapper/GetMode.h"
#include "ethzasl_icp_mapper/GetBoundedMap.h" // FIXME: should that be moved to map_msgs?

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;

class Mapper
{
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	
	typedef typename Nabo::NearestNeighbourSearch<float> NNS;
	typedef typename NNS::SearchType NNSearchType;
		
	ros::NodeHandle& n;
	ros::NodeHandle& pn;
	
	// Subscribers
	ros::Subscriber scanSub;
	ros::Subscriber cloudSub;
	
	// Publishers
	ros::Publisher mapPub;
	ros::Publisher scanPub;
	ros::Publisher outlierPub;
	ros::Publisher odomPub;
	ros::Publisher odomErrorPub;
	
	// Services
	ros::ServiceServer getPointMapSrv;
	ros::ServiceServer saveMapSrv;
	ros::ServiceServer loadMapSrv;
	ros::ServiceServer resetSrv;
	ros::ServiceServer correctPoseSrv;
	ros::ServiceServer setModeSrv;
	ros::ServiceServer getModeSrv;
	ros::ServiceServer getBoundedMapSrv;
	ros::ServiceServer reloadAllYamlSrv;

	// Time
	ros::Time mapCreationTime;
	ros::Time lastPoinCloudTime;
	uint32_t lastPointCloudSeq;

	// libpointmatcher
	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapPreFilters;
	PM::DataPointsFilters mapPostFilters;
	PM::DataPoints *mapPointCloud;
	PM::ICPSequence icp;
	shared_ptr<PM::Transformation> transformation;
	shared_ptr<PM::DataPointsFilter> radiusFilter;
	
	// multi-threading mapper
	#if BOOST_VERSION >= 104100
	typedef boost::packaged_task<PM::DataPoints*> MapBuildingTask;
	typedef boost::unique_future<PM::DataPoints*> MapBuildingFuture;
	boost::thread mapBuildingThread;
	MapBuildingTask mapBuildingTask;
	MapBuildingFuture mapBuildingFuture;
	bool mapBuildingInProgress;
	#endif // BOOST_VERSION >= 104100
	bool processingNewCloud; 

	// Parameters
	bool publishMapTf; 
	bool useConstMotionModel; 
	bool localizing;
	bool mapping;
	bool inverseTransform; // If true, publish the inverse transform, useful to maintain a tree in TF
	int minReadingPointCount;
	int minMapPointCount;
	int inputQueueSize; 
	double minOverlap;
	double maxOverlapToMerge;
	double tfRefreshPeriod;  //!< if set to zero, tf will be publish at the rate of the incoming point cloud messages 
	string sensorFrame;
	string odomFrame;
	string mapFrame;
	string vtkFinalMapName; //!< name of the final vtk map

	const double mapElevation; // initial correction on z-axis //FIXME: handle the full matrix
	
	// Parameters for dynamic filtering
	const float priorDyn; //!< ratio. Prior to be dynamic when a new point is added
	const float priorStatic; //!< ratio. Prior to be static when a new point is added
	const float maxAngle; //!< in rad. Openning angle of a laser beam
	const float eps_a; //!< ratio. Error proportional to the laser distance
	const float eps_d; //!< in meter. Fix error on the laser distance
	const float alpha; //!< ratio. Propability of staying static given that the point was dynamic
	const float beta; //!< ratio. Propability of staying dynamic given that the point was static
	const float maxDyn; //!< ratio. Threshold for which a point will stay dynamic
	const float maxDistNewPoint; //!< in meter. Distance at which a new point will be added in the global map.
	const float sensorMaxRange; //!< in meter. Maximum reading distance of the laser. Used to cut the global map before matching.


	

	PM::TransformationParameters T_odom_to_map;
	PM::TransformationParameters T_localMap_to_map;
	PM::TransformationParameters T_odom_to_scanner;
	boost::thread publishThread;
	boost::mutex publishLock;
	boost::mutex icpMapLock;
	ros::Time publishStamp;
	
	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcaster;

	const float eps;
	
public:
	Mapper(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Mapper();
	
protected:
	void gotScan(const sensor_msgs::LaserScan& scanMsgIn);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
	void processCloud(unique_ptr<DP> cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
	void processNewMapIfAvailable();
	void setMap(DP* newPointCloud);
	DP* updateMap(DP* newPointCloud, const PM::TransformationParameters T_updatedScanner_to_map, bool mapExists);
	void waitForMapBuildingCompleted();
	void updateIcpMap(const DP* newMapPointCloud);
	
	void publishLoop(double publishPeriod);
	void publishTransform();
	void loadExternalParameters();
	
	// Services
	bool getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res);
	bool saveMap(map_msgs::SaveMap::Request &req, map_msgs::SaveMap::Response &res);
	bool loadMap(ethzasl_icp_mapper::LoadMap::Request &req, ethzasl_icp_mapper::LoadMap::Response &res);
	bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool correctPose(ethzasl_icp_mapper::CorrectPose::Request &req, ethzasl_icp_mapper::CorrectPose::Response &res);
	bool setMode(ethzasl_icp_mapper::SetMode::Request &req, ethzasl_icp_mapper::SetMode::Response &res);
	bool getMode(ethzasl_icp_mapper::GetMode::Request &req, ethzasl_icp_mapper::GetMode::Response &res);
	bool getBoundedMap(ethzasl_icp_mapper::GetBoundedMap::Request &req, ethzasl_icp_mapper::GetBoundedMap::Response &res);
	bool reloadallYaml(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

Mapper::Mapper(ros::NodeHandle& n, ros::NodeHandle& pn):
	n(n),
	pn(pn),
	mapPointCloud(0),
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	#if BOOST_VERSION >= 104100
	mapBuildingInProgress(false),
	#endif // BOOST_VERSION >= 104100
	processingNewCloud(false),
	publishMapTf(getParam<bool>("publishMapTf", true)),
	useConstMotionModel(getParam<bool>("useConstMotionModel", false)),
	localizing(getParam<bool>("localizing", true)),
	mapping(getParam<bool>("mapping", true)),
	inverseTransform(getParam<bool>("inverseTransform", false)),
	minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
	minMapPointCount(getParam<int>("minMapPointCount", 500)),
	inputQueueSize(getParam<int>("inputQueueSize", 10)),
	minOverlap(getParam<double>("minOverlap", 0.5)),
	maxOverlapToMerge(getParam<double>("maxOverlapToMerge", 0.9)),
	tfRefreshPeriod(getParam<double>("tfRefreshPeriod", 0.01)),
	sensorFrame(getParam<string>("sensor_frame", "")),
	odomFrame(getParam<string>("odom_frame", "odom")),
	mapFrame(getParam<string>("map_frame", "map")),
	vtkFinalMapName(getParam<string>("vtkFinalMapName", "finalMap.vtk")),
	mapElevation(getParam<double>("mapElevation", 0)),
	priorDyn(getParam<double>("priorDyn", 0.5)),
	priorStatic(1. - priorDyn),
	maxAngle(getParam<double>("maxAngle", 0.02)),
	eps_a(getParam<double>("eps_a", 0.05)),
	eps_d(getParam<double>("eps_d", 0.02)),
	alpha(getParam<double>("alpha", 0.99)),
	beta(getParam<double>("beta", 0.99)),
	maxDyn(getParam<double>("maxDyn", 0.95)),
	maxDistNewPoint(pow(getParam<double>("maxDistNewPoint", 0.1),2)),
	sensorMaxRange(getParam<double>("sensorMaxRange", 80.0)),
	T_odom_to_map(PM::TransformationParameters::Identity(4, 4)),
	T_localMap_to_map(PM::TransformationParameters::Identity(4, 4)),
	T_odom_to_scanner(PM::TransformationParameters::Identity(4, 4)),
	publishStamp(ros::Time::now()),
	tfListener(ros::Duration(30)),
	eps(0.0001)
{


	// Ensure proper states
	if(localizing == false)
		mapping = false;
	if(mapping == true)
		localizing = true;
	
	// set logger
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(make_shared<PointMatcherSupport::ROSLogger>());

	// Load all parameters stored in external files
	loadExternalParameters();

	PM::Parameters params;
	params["dim"] = "-1";
	params["maxDist"] = toParam(sensorMaxRange);

	radiusFilter = PM::get().DataPointsFilterRegistrar.create("MaxDistDataPointsFilter", params);

	// topic initializations
	if (getParam<bool>("subscribe_scan", true))
		scanSub = n.subscribe("scan", inputQueueSize, &Mapper::gotScan, this);
	if (getParam<bool>("subscribe_cloud", true))
		cloudSub = n.subscribe("cloud_in", inputQueueSize, &Mapper::gotCloud, this);

	mapPub = n.advertise<sensor_msgs::PointCloud2>("point_map", 2, true);
	scanPub = n.advertise<sensor_msgs::PointCloud2>("corrected_scan", 2, true);
	outlierPub = n.advertise<sensor_msgs::PointCloud2>("outliers", 2, true);
	odomPub = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	odomErrorPub = n.advertise<nav_msgs::Odometry>("icp_error_odom", 50, true);
	
	// service initializations
	getPointMapSrv = n.advertiseService("dynamic_point_map", &Mapper::getPointMap, this);
	saveMapSrv = pn.advertiseService("save_map", &Mapper::saveMap, this);
	loadMapSrv = pn.advertiseService("load_map", &Mapper::loadMap, this);
	resetSrv = pn.advertiseService("reset", &Mapper::reset, this);
	correctPoseSrv = pn.advertiseService("correct_pose", &Mapper::correctPose, this);
	setModeSrv = pn.advertiseService("set_mode", &Mapper::setMode, this);
	getModeSrv = pn.advertiseService("get_mode", &Mapper::getMode, this);
	getBoundedMapSrv = pn.advertiseService("get_bounded_map", &Mapper::getBoundedMap, this);
	reloadAllYamlSrv= pn.advertiseService("reload_all_yaml", &Mapper::reloadallYaml, this);

	// refreshing tf transform thread
	publishThread = boost::thread(boost::bind(&Mapper::publishLoop, this, tfRefreshPeriod));

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
	if(localizing)
	{
		const ros::Time endScanTime(scanMsgIn.header.stamp + ros::Duration(scanMsgIn.time_increment * (scanMsgIn.ranges.size() - 1)));
		try
		{
			unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scanMsgIn, &tfListener, scanMsgIn.header.frame_id))); //odomFrame

			processCloud(move(cloud), scanMsgIn.header.frame_id, endScanTime, scanMsgIn.header.seq);
		}
		catch(tf2::ConnectivityException)
		{
			ROS_ERROR_STREAM("Multiple tree... ignoring scan");
		}
		catch(tf2::TransformException &e)
		{
			ROS_ERROR_STREAM("Fix me..." << e.what() );
		}
	}
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	if(localizing)
	{
		unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
		
		
		// TEST for UTIAS work on velodyne
		// TODO: move that to a libpointmatcher filter
		//const int maxNbPoints = 38000; // 70000 or 38000
		//if(cloud->getNbPoints() >= maxNbPoints)
		//{
	//		cloud->features = cloud->features.leftCols(maxNbPoints); 
	//		cloud->descriptors= cloud->descriptors.leftCols(maxNbPoints);
		//}
		processCloud(move(cloud), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
	}
}

struct BoolSetter
{
public:
	bool toSetValue;
	BoolSetter(bool& target, bool toSetValue):
		toSetValue(toSetValue),
		target(target)
	{}
	~BoolSetter()
	{
		target = toSetValue;
	}
protected:
	bool& target;
};

void Mapper::processCloud(unique_ptr<DP> newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{

	processingNewCloud = true;
	BoolSetter stopProcessingSetter(processingNewCloud, false);
	
	// If the sensor frame was not set by the user, use default
	if(sensorFrame == "")
	{
		this->sensorFrame = scannerFrame;
	}

	// if the future has completed, use the new map
	
	//TODO: investigate if we need that
	processNewMapIfAvailable();
	
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
	timer t;
	
	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("[ICP] I found no good points in the cloud");
		return;
	}
	
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

	// This need to be depreciated, there is addTime for those field in pm
	if(!(newPointCloud->descriptorExists("stamps_Msec") && newPointCloud->descriptorExists("stamps_sec") && newPointCloud->descriptorExists("stamps_nsec")))
	{
		const float Msec = round(stamp.sec/1e6);
		const float sec = round(stamp.sec - Msec*1e6);
		const float nsec = round(stamp.nsec);

		const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);
		const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
		const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
		newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
		newPointCloud->addDescriptor("stamps_sec", desc_sec);
		newPointCloud->addDescriptor("stamps_nsec", desc_nsec);

		//cout << "Adding time" << endl;
		
	}

	// Ensure a minimum amount of point before filtering
	int ptsCount = newPointCloud->getNbPoints();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("[ICP] Not enough points in newPointCloud: only " << ptsCount << " pts.");
		return;
	}


	{
		timer t; // Print how long take the algo
		
		// Apply filters to incoming cloud, in scanner coordinates
		inputFilters.apply(*newPointCloud);
		
		ROS_INFO_STREAM("[ICP] Input filters took " << t.elapsed() << " [s]");
	}

	string reason;
	// Initialize the transformation to identity if empty
 	if(!icp.hasMap())
 	{
		// we need to know the dimensionality of the point cloud to initialize properly
		publishLock.lock();
		T_odom_to_map = PM::TransformationParameters::Identity(dimp1, dimp1);
		T_localMap_to_map = PM::TransformationParameters::Identity(dimp1, dimp1);
		T_odom_to_scanner = PM::TransformationParameters::Identity(dimp1, dimp1);
		publishLock.unlock();
	}
	else
	{
		//Ensure that all global transforms don't cumulate floating error
		T_odom_to_map = transformation->correctParameters(T_odom_to_map);
		T_localMap_to_map = transformation->correctParameters(T_localMap_to_map);
		T_odom_to_scanner = transformation->correctParameters(T_odom_to_scanner);
	}


	// Fetch transformation from scanner to odom
	// Note: we don't need to actively wait for transform here. It is already waited in transformListenerToEigenMatrix()
	try
	{
		T_odom_to_scanner = PointMatcher_ros::eigenMatrixToDim<float>(
				PointMatcher_ros::transformListenerToEigenMatrix<float>(
				tfListener,
				scannerFrame, // to
				odomFrame, // from
				stamp
			), dimp1);
	}
	catch(tf::ExtrapolationException e)
	{
		ROS_ERROR_STREAM("Extrapolation Exception. stamp = "<< stamp << " now = " << ros::Time::now() << " delta = " << ros::Time::now() - stamp << endl << e.what() );
		return;
	}
	catch ( ... )
	{
		// everything else
		ROS_ERROR_STREAM("Unexpected exception... ignoring scan");
		return;
	}

	const PM::TransformationParameters T_scanner_to_map = T_odom_to_map * T_odom_to_scanner.inverse();

	// Recuring need to see those transformations...
	ROS_DEBUG_STREAM("[ICP] T_odom_to_scanner(" << odomFrame<< " to " << scannerFrame << "):\n" << T_odom_to_scanner);
	ROS_DEBUG_STREAM("[ICP] T_odom_to_map(" << odomFrame<< " to " << mapFrame << "):\n" << T_odom_to_map);
	ROS_DEBUG_STREAM("[ICP] T_scanner_to_map (" << scannerFrame << " to " << mapFrame << "):\n" << T_scanner_to_map);

	const PM::TransformationParameters T_scanner_to_localMap = transformation->correctParameters(T_localMap_to_map.inverse() * T_scanner_to_map);

	// Ensure a minimum amount of point after filtering
	ptsCount = newPointCloud->getNbPoints();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("[ICP] Not enough points in newPointCloud: only " << ptsCount << " pts.");
		return;
	}

	// Initialize the map if empty
 	if(!icp.hasMap())
 	{
		ROS_INFO_STREAM("[MAP] Creating an initial map");
		mapCreationTime = stamp;
		setMap(updateMap(newPointCloud.release(), T_scanner_to_map, false));
		// we must not delete newPointCloud because we just stored it in the mapPointCloud
		return;
	}
	
	// Check dimension
	if (newPointCloud->getEuclideanDim() != icp.getPrefilteredInternalMap().getEuclideanDim())
	{
		ROS_ERROR_STREAM("[ICP] Dimensionality missmatch: incoming cloud is " << newPointCloud->getEuclideanDim() << " while map is " << icp.getPrefilteredInternalMap().getEuclideanDim());
		return;
	}
	
	try 
	{
		// Apply ICP
		PM::TransformationParameters T_updatedScanner_to_map;
		PM::TransformationParameters T_updatedScanner_to_localMap;
		
		ROS_INFO_STREAM("[ICP] Computing - reading: " << newPointCloud->getNbPoints() << ", reference: " << icp.getInternalMap().getNbPoints() );

		icpMapLock.lock();
		T_updatedScanner_to_localMap = icp(*newPointCloud, T_scanner_to_localMap);
		icpMapLock.unlock();

		T_updatedScanner_to_map = T_localMap_to_map * T_updatedScanner_to_localMap;


		ROS_DEBUG_STREAM("[ICP] T_updatedScanner_to_map:\n" << T_updatedScanner_to_map);
		ROS_DEBUG_STREAM("[ICP] T_updatedScanner_to_localMap:\n" << T_updatedScanner_to_localMap);
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		ROS_INFO_STREAM("[ICP] Overlap: " << estimatedOverlap);
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("[ICP] Estimated overlap too small, ignoring ICP correction!");
			return;
		}
		
		// Compute tf
		publishStamp = stamp;
		publishLock.lock();

		// Update old transform
		T_odom_to_map = T_updatedScanner_to_map * T_odom_to_scanner;
		
		// Publish tf
		if(publishMapTf == true)
		{
			
			if (inverseTransform) 
			{
				tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_odom_to_map.inverse(), odomFrame, mapFrame, stamp));
			} 
			else 
			{
				tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_odom_to_map, mapFrame, odomFrame, stamp));
			}		
		}

		publishLock.unlock();
		processingNewCloud = false;
		
		ROS_DEBUG_STREAM("[ICP] T_odom_to_map:\n" << T_odom_to_map);

		// Publish odometry
		if (odomPub.getNumSubscribers())
		{
			// Not sure that the transformation represents the odometry
			odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(T_updatedScanner_to_map, mapFrame, stamp));
		}
		// Publish error on odometry
		if (odomErrorPub.getNumSubscribers())
			odomErrorPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(T_odom_to_map, mapFrame, stamp));

		

		// check if news points should be added to the map
		if (
			((estimatedOverlap < maxOverlapToMerge) || (icp.getPrefilteredInternalMap().features.cols() < minMapPointCount)) &&(!mapBuildingInProgress)
		)
		{
			// make sure we process the last available map
			mapCreationTime = stamp;

			ROS_INFO("[MAP] Adding new points in a separate thread");
			
			mapBuildingTask = MapBuildingTask(boost::bind(&Mapper::updateMap, this, newPointCloud.release(), T_updatedScanner_to_map, true));
			mapBuildingFuture = mapBuildingTask.get_future();
			mapBuildingThread = boost::thread(boost::move(boost::ref(mapBuildingTask)));
			mapBuildingThread.detach(); // We don't care about joining this one
			sched_yield();
			mapBuildingInProgress = true;
		}
		else
		{
			cerr << "SKIPPING MAP" << endl;
			cerr << "estimatedOverlap < maxOverlapToMerge: " << (estimatedOverlap < maxOverlapToMerge) << endl;
			cerr << "(icp.getInternalMap().features.cols() < minMapPointCount): " << icp.getInternalMap().features.cols() << " < " << minMapPointCount << " = " << (icp.getInternalMap().features.cols() < minMapPointCount) << endl;
			cerr << "mapBuildingInProgress: " << mapBuildingInProgress << endl;
			
			bool stateLock = publishLock.try_lock();
			if(stateLock) publishLock.unlock();
			cerr << "publishLock.try_lock(): " << stateLock << endl;
			
			stateLock = icpMapLock.try_lock();
			if(stateLock) icpMapLock.unlock();
			cerr << "icpMapLock.try_lock(): " << stateLock << endl;

			cerr << "mapBuildingFuture.has_value(): " << mapBuildingFuture.has_value() << endl;

		}
		
	}
	catch (PM::ConvergenceError error)
	{
		icpMapLock.unlock();
		ROS_ERROR_STREAM("[ICP] failed to converge: " << error.what());
		newPointCloud->save("error_read.vtk");
		icp.getPrefilteredMap().save("error_ref.vtk");
		return;
	}
	
	//Statistics about time and real-time capability
	int realTimeRatio = 100*t.elapsed() / (stamp.toSec()-lastPoinCloudTime.toSec());
	realTimeRatio *= seq - lastPointCloudSeq;

	ROS_INFO_STREAM("[TIME] Total ICP took: " << t.elapsed() << " [s]");
	if(realTimeRatio < 80)
		ROS_INFO_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
	else
		ROS_WARN_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

	lastPoinCloudTime = stamp;
	lastPointCloudSeq = seq;
}

void Mapper::processNewMapIfAvailable()
{
	#if BOOST_VERSION >= 104100
	if (mapBuildingInProgress && mapBuildingFuture.has_value())
	{
		ROS_INFO_STREAM("[MAP] Computation in thread done");
		setMap(mapBuildingFuture.get());
		mapBuildingInProgress = false;
	}
	#endif // BOOST_VERSION >= 104100
}

void Mapper::setMap(DP* newMapPointCloud)
{

	// delete old map
	if (mapPointCloud && mapPointCloud != newMapPointCloud)
		delete mapPointCloud;
	
	// set new map
	mapPointCloud = newMapPointCloud;

	// update ICP map
	updateIcpMap(mapPointCloud);
	
	// Publish map point cloud
	// FIXME this crash when used without descriptor
	publishLock.lock();
	if (mapPub.getNumSubscribers() && mapping)
	{
		ROS_INFO_STREAM("[MAP] publishing " << mapPointCloud->getNbPoints() << " points");
		mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, mapFrame, mapCreationTime));
	}
	publishLock.unlock();
}

void Mapper::updateIcpMap(const DP* newMapPointCloud)
{
	ros::Time time_current = ros::Time::now();
	//TODO check if T_odom_to_scanner can be reused
	try
	{
		//tfListener.waitForTransform(sensorFrame, odomFrame, time_current, ros::Duration(3.0));

		// Fetching current transformation to the sensor
	
		//const PM::TransformationParameters T_odom_to_scanner = 
		//	PointMatcher_ros::eigenMatrixToDim<float>(
		//			PointMatcher_ros::transformListenerToEigenMatrix<float>(
		//				tfListener,
		//				sensorFrame, 
		//				odomFrame,
		//				time_current
		//				), mapPointCloud->getHomogeneousDim());

		// Move the global map to the scanner pose
		const PM::TransformationParameters T_scanner_to_map = this->T_odom_to_map * T_odom_to_scanner.inverse();
		DP localMap = transformation->compute(*newMapPointCloud, T_scanner_to_map.inverse());

		// Cut points in a radius of the parameter sensorMaxRange
		radiusFilter->inPlaceFilter(localMap);

		icpMapLock.lock();
		// Update the transformation to the local map
		this->T_localMap_to_map = T_scanner_to_map;

		icp.setMap(localMap);
		icpMapLock.unlock();
	}
	catch ( ... )
	{
		// everything else
		ROS_ERROR_STREAM("Unexpected exception... ignoring scan");
		return;
	}
}

Mapper::DP* Mapper::updateMap(DP* newPointCloud, const PM::TransformationParameters T_updatedScanner_to_map, bool mapExists)
{
	timer t;

	try
	{
		// Prepare empty field if not existing
		if(newPointCloud->descriptorExists("probabilityStatic") == false)
		{
			//newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Zero(1, newPointCloud->features.cols()));
			newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorStatic));
		}
		
		if(newPointCloud->descriptorExists("probabilityDynamic") == false)
		{
			//newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Zero(1, newPointCloud->features.cols()));
			newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorDyn));
		}
		
		if(newPointCloud->descriptorExists("debug") == false)
		{
			newPointCloud->addDescriptor("debug", PM::Matrix::Zero(1, newPointCloud->features.cols()));
		}

		

		if (!mapExists) 
		{
			ROS_INFO_STREAM( "[MAP] Initial map, only filtering points");
			*newPointCloud = transformation->compute(*newPointCloud, T_updatedScanner_to_map); 
			mapPostFilters.apply(*newPointCloud);

			return newPointCloud;
		}


		// Early out if no map modification is wanted
		if(!mapping)
		{
			ROS_INFO_STREAM("[MAP] Skipping modification of the map");
			return mapPointCloud;
		}
		
		
		const int dimp1(newPointCloud->features.rows());
		const int mapPtsCount(mapPointCloud->getNbPoints());
		const int readPtsCount(newPointCloud->getNbPoints());
		const int dim = mapPointCloud->getEuclideanDim();
		
		// Build a range image of the reading point cloud (local coordinates)
		PM::Matrix radius_reading = newPointCloud->features.topRows(dimp1-1).colwise().norm();

		PM::Matrix angles_reading(2, readPtsCount); // 0=inclination, 1=azimuth

		// No atan in Eigen, so we are for to loop through it...
		for(int i=0; i<readPtsCount; i++)
		{
			angles_reading(0,i) = 0;
            if (dimp1 == 4) { // 3D only
                const float ratio = newPointCloud->features(2,i)/radius_reading(0,i);
                angles_reading(0,i) = acos(ratio);
            }
			angles_reading(1,i) = atan2(newPointCloud->features(1,i), newPointCloud->features(0,i));
		}

		std::shared_ptr<NNS> featureNNS;
		featureNNS.reset( NNS::create(angles_reading));


		// Transform the global map in local coordinates
		DP mapLocalFrameCut = transformation->compute(*mapPointCloud, T_updatedScanner_to_map.inverse());
		

		// Remove points out of sensor range
		PM::Matrix globalId(1, mapPtsCount); 

		int mapCutPtsCount= 0;
		for (int i = 0; i < mapPtsCount; i++)
		{
			if (mapLocalFrameCut.features.col(i).head(dimp1-1).norm() < sensorMaxRange)
			{
				mapLocalFrameCut.setColFrom(mapCutPtsCount, mapLocalFrameCut, i);
				globalId(0,mapCutPtsCount) = i;
				mapCutPtsCount++;
			}
		}

		mapLocalFrameCut.conservativeResize(mapCutPtsCount);

		PM::Matrix radius_map = mapLocalFrameCut.features.topRows(dimp1-1).colwise().norm();

		PM::Matrix angles_map(2, mapCutPtsCount); // 0=inclination, 1=azimuth

		// No atan in Eigen, so we need to loop through it...
		for(int i=0; i<mapCutPtsCount; i++)
		{
			angles_map(0,i) = 0;
            if (dimp1 == 4) { // 3D
                const float ratio = mapLocalFrameCut.features(2,i)/radius_map(0,i);
                angles_map(0,i) = acos(ratio);
            }

			angles_map(1,i) = atan2(mapLocalFrameCut.features(1,i), mapLocalFrameCut.features(0,i));
		}

		// Look for NN in spherical coordinates
		Matches::Dists dists(1,mapCutPtsCount);
		Matches::Ids ids(1,mapCutPtsCount);
		
		featureNNS->knn(angles_map, ids, dists, 1, 0, NNS::ALLOW_SELF_MATCH, maxAngle);

		// Define views on descriptors
		DP::View viewOn_Msec_overlap = newPointCloud->getDescriptorViewByName("stamps_Msec");
		DP::View viewOn_sec_overlap = newPointCloud->getDescriptorViewByName("stamps_sec");
		DP::View viewOn_nsec_overlap = newPointCloud->getDescriptorViewByName("stamps_nsec");

		DP::View viewOnProbabilityStatic = mapPointCloud->getDescriptorViewByName("probabilityStatic");
		DP::View viewOnProbabilityDynamic = mapPointCloud->getDescriptorViewByName("probabilityDynamic");
		DP::View viewDebug = mapPointCloud->getDescriptorViewByName("debug");
		
		DP::View viewOn_normals_map = mapLocalFrameCut.getDescriptorViewByName("normals");
		DP::View viewOn_Msec_map = mapPointCloud->getDescriptorViewByName("stamps_Msec");
		DP::View viewOn_sec_map = mapPointCloud->getDescriptorViewByName("stamps_sec");
		DP::View viewOn_nsec_map = mapPointCloud->getDescriptorViewByName("stamps_nsec");
		
		
		viewDebug = PM::Matrix::Zero(1, mapPtsCount);
		for(int i=0; i < mapCutPtsCount; i++)
		{
			if(dists(i) != numeric_limits<float>::infinity())
			{
				const int readId = ids(0,i);
				const int mapId = globalId(0,i);
				
				// in local coordinates
				const Eigen::VectorXf readPt = newPointCloud->features.col(readId).head(dimp1-1);
				const Eigen::VectorXf mapPt = mapLocalFrameCut.features.col(i).head(dimp1-1);
				const Eigen::VectorXf mapPt_n = mapPt.normalized();
				const float delta = (readPt - mapPt).norm();
				const float d_max = eps_a * readPt.norm();

				const Eigen::VectorXf normal_map = viewOn_normals_map.col(i);
				
				// Weight for dynamic elements
				const float w_v = eps + (1. - eps)*fabs(normal_map.dot(mapPt_n));
				const float w_d1 =  eps + (1. - eps)*(1. - sqrt(dists(i))/maxAngle);
				
				
				const float offset = delta - eps_d;
				float w_d2 = 1.;
				if(delta < eps_d || mapPt.norm() > readPt.norm())
				{
					w_d2 = eps;
				}
				else 
				{
					if (offset < d_max)
					{
						w_d2 = eps + (1 - eps )*offset/d_max;
					}
				}

				float w_p2 = eps;
				if(delta < eps_d)
				{
					w_p2 = 1;
				}
				else
				{
					if(offset < d_max)
					{
						w_p2 = eps + (1. - eps)*(1. - offset/d_max);
					}
				}


				// We don't update point behind the reading
				if((readPt.norm() + eps_d + d_max) >= mapPt.norm())
				{
					const float lastDyn = viewOnProbabilityDynamic(0,mapId);
					const float lastStatic = viewOnProbabilityStatic(0, mapId);

					const float c1 = (1 - (w_v*w_d1));
					const float c2 = w_v*w_d1;
					

					//Lock dynamic point to stay dynamic under a threshold
					if(lastDyn < maxDyn)
					{
						viewOnProbabilityDynamic(0,mapId) = c1*lastDyn + c2*w_d2*((1 - alpha)*lastStatic + beta*lastDyn);
						viewOnProbabilityStatic(0, mapId) = c1*lastStatic + c2*w_p2*(alpha*lastStatic + (1 - beta)*lastDyn);
					}
					else
					{
						viewOnProbabilityStatic(0,mapId) = eps;
						viewOnProbabilityDynamic(0,mapId) = 1-eps;
					}
					
					
					
					// normalization
					const float sumZ = viewOnProbabilityDynamic(0,mapId) + viewOnProbabilityStatic(0, mapId);
					assert(sumZ >= eps);	
					
					viewOnProbabilityDynamic(0,mapId) /= sumZ;
					viewOnProbabilityStatic(0,mapId) /= sumZ;
					
					viewDebug(0,mapId) = w_d2;


					//TODO use the new time structure
					// Refresh time
					viewOn_Msec_map(0,mapId) = viewOn_Msec_overlap(0,readId);	
					viewOn_sec_map(0,mapId) = viewOn_sec_overlap(0,readId);	
					viewOn_nsec_map(0,mapId) = viewOn_nsec_overlap(0,readId);	
				}


			}
		}

		// Generate temporary map for density computation
		DP tmp_map = mapLocalFrameCut; 
		tmp_map.concatenate(*newPointCloud);

		// build and populate NNS
		featureNNS.reset( NNS::create(tmp_map.features, tmp_map.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
		
		PM::Matches matches_overlap(
			Matches::Dists(1, readPtsCount),
			Matches::Ids(1, readPtsCount)
		);
		

		featureNNS->knn(newPointCloud->features, matches_overlap.ids, matches_overlap.dists, 1, 0);
		
		DP overlap(newPointCloud->createSimilarEmpty());
		DP no_overlap(newPointCloud->createSimilarEmpty());

		int ptsOut = 0;
		int ptsIn = 0;
		for (int i = 0; i < readPtsCount; ++i)
		{
			if (matches_overlap.dists(i) > maxDistNewPoint)
			{
				no_overlap.setColFrom(ptsOut, *newPointCloud, i);
				ptsOut++;
			}
			else
			{
				overlap.setColFrom(ptsIn, *newPointCloud, i);
				ptsIn++;
			}
		}

		no_overlap.conservativeResize(ptsOut);
		overlap.conservativeResize(ptsIn);

		// Publish outliers
		//if (outlierPub.getNumSubscribers())
		//{
			//outlierPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(no_overlap, mapFrame, mapCreationTime));
		//}

		// Initialize descriptors
		no_overlap.addDescriptor("probabilityStatic", PM::Matrix::Constant(1, no_overlap.features.cols(), priorStatic));
		no_overlap.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, no_overlap.features.cols(), priorDyn));
		no_overlap.addDescriptor("debug", PM::Matrix::Zero(1, no_overlap.features.cols()));

		// shrink the newPointCloud to the new information
		*newPointCloud = no_overlap;
		
		// Correct new points using ICP result
		*newPointCloud = transformation->compute(*newPointCloud, T_updatedScanner_to_map);
		
		// Merge point clouds to map
		newPointCloud->concatenate(*mapPointCloud);
		mapPostFilters.apply(*newPointCloud);
	}
	catch(DP::InvalidField e)
	{
		ROS_ERROR_STREAM(e.what());
		abort();
	}
	catch (const std::exception &exc)
	{
		// catch anything thrown within try block that derives from std::exception
		std::cerr << exc.what();
		abort();
	}
	
	ROS_INFO_STREAM("[TIME][MAP] New map available (" << newPointCloud->features.cols() << " pts), update took " << t.elapsed() << " [s]");
	
	return newPointCloud;
}

void Mapper::waitForMapBuildingCompleted()
{
	#if BOOST_VERSION >= 104100
	if (mapBuildingInProgress)
	{
		// we wait for now, in future we should kill it
		mapBuildingFuture.wait();
		mapBuildingInProgress = false;
	}
	#endif // BOOST_VERSION >= 104100
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
	if(processingNewCloud == false && publishMapTf == true)
	{
		publishLock.lock();
    ros::Time stamp = ros::Time::now();
		// Note: we use now as timestamp to refresh the tf and avoid other buffer to be empty
		if (inverseTransform) {
        tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_odom_to_map.inverse(), odomFrame, mapFrame, stamp));
    } else {
        tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_odom_to_map, mapFrame, odomFrame, stamp));
    }		
		
		publishLock.unlock();
	}
}

bool Mapper::getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res)
{
	if (!mapPointCloud)
		return false;
	
	// FIXME: do we need a mutex here?
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
		ROS_ERROR_STREAM("Unable to save: " << e.what());
		return false;
	}
	
	ROS_INFO_STREAM("[MAP] saved at " <<  req.filename.data << " with " << mapPointCloud->features.cols() << " points.");
	return true;
}

bool Mapper::loadMap(ethzasl_icp_mapper::LoadMap::Request &req, ethzasl_icp_mapper::LoadMap::Response &res)
{
	waitForMapBuildingCompleted();
	
	DP* cloud(new DP(DP::load(req.filename.data)));

	// Print new map information
	const int dim = cloud->features.rows();
	const int nbPts = cloud->features.cols();
	ROS_INFO_STREAM("[MAP] Loading " << dim-1 << "D point cloud (" << req.filename.data << ") with " << nbPts << " points.");

	ROS_INFO_STREAM("  With descriptors:");
	for(int i=0; i< cloud->descriptorLabels.size(); i++)
	{
		ROS_INFO_STREAM("    - " << cloud->descriptorLabels[i].text); 
	}

	//reset transformation
	publishLock.lock();
	T_odom_to_map = PM::TransformationParameters::Identity(dim,dim);
	T_localMap_to_map = PM::TransformationParameters::Identity(dim,dim);
	T_odom_to_scanner = PM::TransformationParameters::Identity(dim,dim);
	
	//ISER
	//T_odom_to_map(2,3) = mapElevation;
	publishLock.unlock();

	//TODO: check that...
	mapping = true;
	setMap(updateMap(cloud, PM::TransformationParameters::Identity(dim,dim), false));
	mapping = false;
	
	return true;
}

bool Mapper::reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	waitForMapBuildingCompleted();
	
	// note: no need for locking as we do ros::spin(), to update if we go for multi-threading
	publishLock.lock();
	// WARNING: this will break in 2D
	T_odom_to_map = PM::TransformationParameters::Identity(4,4);
	T_localMap_to_map = PM::TransformationParameters::Identity(4,4);
	T_odom_to_scanner = PM::TransformationParameters::Identity(4,4);
	publishLock.unlock();

	icp.clearMap();
	
	return true;
}

bool Mapper::correctPose(ethzasl_icp_mapper::CorrectPose::Request &req, ethzasl_icp_mapper::CorrectPose::Response &res)
{
	publishLock.lock();
	const int dim = mapPointCloud->getHomogeneousDim();

	try
	{
		T_odom_to_map = PointMatcher_ros::eigenMatrixToDim<float>(PointMatcher_ros::odomMsgToEigenMatrix<float>(req.odom), dim);


		const PM::TransformationParameters T_odom_to_scanner = 
			PointMatcher_ros::eigenMatrixToDim<float>(
					PointMatcher_ros::transformListenerToEigenMatrix<float>(
						tfListener,
						sensorFrame, 
						odomFrame,
						ros::Time::now()
						), dim);

		const PM::TransformationParameters T_scanner_to_map = T_odom_to_map * T_odom_to_scanner.inverse();


		// update ICP map
		updateIcpMap(mapPointCloud);

		//ISER
		/*
			 {
		// remove roll and pitch
		T_odom_to_map(2,0) = 0; 
		T_odom_to_map(2,1) = 0; 
		T_odom_to_map(2,2) = 1; 
		T_odom_to_map(0,2) = 0; 
		T_odom_to_map(1,2) = 0;
		T_odom_to_map(2,3) = mapElevation; //z
		}*/

		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_odom_to_map, mapFrame, odomFrame, ros::Time::now()));

	}
	catch ( ... )
	{
		// everything else
		publishLock.unlock();
		ROS_ERROR_STREAM("Unexpected exception... ignoring scan");
		return false;
	}

	publishLock.unlock();

	return true;
}

bool Mapper::setMode(ethzasl_icp_mapper::SetMode::Request &req, ethzasl_icp_mapper::SetMode::Response &res)
{
	// Impossible states
	if(req.localize == false && req.map == true)
		return false;

	localizing = req.localize;
	mapping = req.map;
	
	return true;
}

bool Mapper::getMode(ethzasl_icp_mapper::GetMode::Request &req, ethzasl_icp_mapper::GetMode::Response &res)
{
	res.localize = localizing;
	res.map = mapping;
	return true;
}



bool Mapper::getBoundedMap(ethzasl_icp_mapper::GetBoundedMap::Request &req, ethzasl_icp_mapper::GetBoundedMap::Response &res)
{
	if (!mapPointCloud)
		return false;

	const float max_x = req.topRightCorner.x;
	const float max_y = req.topRightCorner.y;
	const float max_z = req.topRightCorner.z;

	const float min_x = req.bottomLeftCorner.x;
	const float min_y = req.bottomLeftCorner.y;
	const float min_z = req.bottomLeftCorner.z;


	tf::StampedTransform stampedTr;
	
	Eigen::Affine3d eigenTr;
	tf::poseMsgToEigen(req.mapCenter, eigenTr);
	Eigen::MatrixXf T = eigenTr.matrix().inverse().cast<float>();
	//const Eigen::MatrixXf T = eigenTr.matrix().cast<float>();

	T = transformation->correctParameters(T);

		
	// FIXME: do we need a mutex here?
	const DP centeredPointCloud = transformation->compute(*mapPointCloud, T); 
	DP cutPointCloud = centeredPointCloud.createSimilarEmpty();

	int newPtCount = 0;
	for(int i=0; i < centeredPointCloud.features.cols(); i++)
	{
		const float x = centeredPointCloud.features(0,i);
		const float y = centeredPointCloud.features(1,i);
		const float z = centeredPointCloud.features(2,i);
		
		if(x < max_x && x > min_x &&
			 y < max_y && y > min_y &&
		   z < max_z && z > min_z 	)
		{
			cutPointCloud.setColFrom(newPtCount, centeredPointCloud, i);
			newPtCount++;	
		}
	}

	ROS_INFO_STREAM("Extract " << newPtCount << " points from the map");
	
	cutPointCloud.conservativeResize(newPtCount);
	cutPointCloud = transformation->compute(cutPointCloud, T.inverse()); 

	
	// Send the resulting point cloud in ROS format
	res.boundedMap = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(cutPointCloud, mapFrame, ros::Time::now());
	return true;
}

void Mapper::loadExternalParameters()
{
	
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
}

bool Mapper::reloadallYaml(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	loadExternalParameters();	
	ROS_INFO_STREAM("Parameters reloaded");

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
