#include <fstream>
#include <iostream>
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

class Mapper
{
	typedef PointMatcher<float> PM;
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
	ros::Publisher debugPub;
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

	// Time
	ros::Time mapCreationTime;
	ros::Time lastPoinCloudTime;

	// libpointmatcher
	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapPreFilters;
	PM::DataPointsFilters mapPostFilters;
	PM::DataPoints *mapPointCloud;
	PM::ICPSequence icp;
	unique_ptr<PM::Transformation> transformation;
	
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
	int minReadingPointCount;
	int minMapPointCount;
	int inputQueueSize; 
	double minOverlap;
	double maxOverlapToMerge;
	double tfRefreshPeriod;  //!< if set to zero, tf will be publish at the rate of the incoming point cloud messages 
	string odomFrame;
	string mapFrame;
	string vtkFinalMapName; //!< name of the final vtk map

	const double mapElevation; // initial correction on z-axis //FIXME: handle the full matrix
	
	// Parameters for dynamic filtering
	const float priorStatic; //!< ratio. Prior to be static when a new point is added
	const float priorDyn; //!< ratio. Prior to be dynamic when a new point is added
	const float maxAngle; //!< in rad. Openning angle of a laser beam
	const float eps_a; //!< ratio. Error proportional to the laser distance
	const float eps_d; //!< in meter. Fix error on the laser distance
	const float alpha; //!< ratio. Propability of staying static given that the point was dynamic
	const float beta; //!< ratio. Propability of staying dynamic given that the point was static
	const float maxDyn; //!< ratio. Threshold for which a point will stay dynamic
	const float maxDistNewPoint; //!< in meter. Distance at which a new point will be added in the global map.
	const float sensorMaxRange; //!< in meter. Radius in which the global map needs to be updated when a new scan arrived.


  // Path information
  // FIXME: this would be better in a structure
  struct ScanInfo
  {
    PM::TransformationParameters pose; //!< pose after ICP correction
    PM::TransformationParameters error; //!< error corrected by point cloud alignment
    ros::Time timeStamp; //!< time at which the point cloud was created
  };

  std::vector<ScanInfo> path; //!< vector representing a path

	//std::vector<PM::TransformationParameters> path; //!< vector of poses representing the path after ICP correction.
	//std::vector<PM::TransformationParameters> errors; //!< vector of poses representing the correction applied by ICP.

	PM::TransformationParameters TOdomToMap;
	boost::thread publishThread;
	boost::mutex publishLock;
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
	DP* updateMap(DP* newPointCloud, const PM::TransformationParameters Ticp, bool updateExisting);
	void waitForMapBuildingCompleted();
	
	void publishLoop(double publishPeriod);
	void publishTransform();

  string getTransParamCsvHeader(const PM::TransformationParameters T, const string prefix);
  string serializeTransParamCSV(const PM::TransformationParameters T, const bool getHeader = false, const string prefix = "");
  void savePathToCsv(string fileName);
	
	// Services
	bool getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res);
	bool saveMap(map_msgs::SaveMap::Request &req, map_msgs::SaveMap::Response &res);
	bool loadMap(ethzasl_icp_mapper::LoadMap::Request &req, ethzasl_icp_mapper::LoadMap::Response &res);
	bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool correctPose(ethzasl_icp_mapper::CorrectPose::Request &req, ethzasl_icp_mapper::CorrectPose::Response &res);
	bool setMode(ethzasl_icp_mapper::SetMode::Request &req, ethzasl_icp_mapper::SetMode::Response &res);
	bool getMode(ethzasl_icp_mapper::GetMode::Request &req, ethzasl_icp_mapper::GetMode::Response &res);
	bool getBoundedMap(ethzasl_icp_mapper::GetBoundedMap::Request &req, ethzasl_icp_mapper::GetBoundedMap::Response &res);
};

string Mapper::getTransParamCsvHeader(const PM::TransformationParameters T, const string prefix)
{
  return serializeTransParamCSV(T, true, prefix);
  
}

string Mapper::serializeTransParamCSV(const PM::TransformationParameters T, const bool getHeader, const string prefix)
{

  std::stringstream stream;
  for(int col=0; col < T.cols(); col++)
  {
    for(int row=0; row < T.rows(); row++)
    {
      if(getHeader)
      {
        stream << prefix << row << col;
      }
      else
      {
        stream << T(row,col);
      }

      if((col != (T.cols()-1)) || (row != (T.rows()-1)))
      {
        stream << ", ";
      }
    } 
  }

  return stream.str();
}



//TODO: move that at the end
void Mapper::savePathToCsv(string fileName)
{
  ofstream file;
  file.open (fileName);
  
  // Save header
  file << getTransParamCsvHeader(path[0].pose, "T") << ", ";
  file << getTransParamCsvHeader(path[0].error, "Delta") << ", ";
  file << "secs, nsecs\n";

  for(size_t k=0; k < path.size(); ++k)
  {
    file << serializeTransParamCSV(path[k].pose) << ", ";
    file << serializeTransParamCSV(path[k].error) << ", ";
    file << path[k].timeStamp.sec << ", " << path[k].timeStamp.nsec << "\n";
  }

  file.close();

}

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
	minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
	minMapPointCount(getParam<int>("minMapPointCount", 500)),
	inputQueueSize(getParam<int>("inputQueueSize", 10)),
	minOverlap(getParam<double>("minOverlap", 0.5)),
	maxOverlapToMerge(getParam<double>("maxOverlapToMerge", 0.9)),
	tfRefreshPeriod(getParam<double>("tfRefreshPeriod", 0.01)),
	odomFrame(getParam<string>("odom_frame", "odom")),
	mapFrame(getParam<string>("map_frame", "map")),
	vtkFinalMapName(getParam<string>("vtkFinalMapName", "finalMap.vtk")),
	mapElevation(getParam<double>("mapElevation", 0)),
	priorStatic(getParam<double>("priorStatic", 0.5)),
	priorDyn(getParam<double>("priorDyn", 0.5)),
	maxAngle(getParam<double>("maxAngle", 0.02)),
	eps_a(getParam<double>("eps_a", 0.05)),
	eps_d(getParam<double>("eps_d", 0.02)),
	alpha(getParam<double>("alpha", 0.99)),
	beta(getParam<double>("beta", 0.99)),
	maxDyn(getParam<double>("maxDyn", 0.95)),
	maxDistNewPoint(pow(getParam<double>("maxDistNewPoint", 0.1),2)),
	sensorMaxRange(getParam<double>("sensorMaxRange", 80.0)),
	TOdomToMap(PM::TransformationParameters::Identity(4, 4)),
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
	debugPub = n.advertise<sensor_msgs::PointCloud2>("debugPointCloud", 2, true);
	odomPub = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	odomErrorPub = n.advertise<nav_msgs::Odometry>("icp_error_odom", 50, true);
	getPointMapSrv = n.advertiseService("dynamic_point_map", &Mapper::getPointMap, this);
	saveMapSrv = pn.advertiseService("save_map", &Mapper::saveMap, this);
	loadMapSrv = pn.advertiseService("load_map", &Mapper::loadMap, this);
	resetSrv = pn.advertiseService("reset", &Mapper::reset, this);
	correctPoseSrv = pn.advertiseService("correct_pose", &Mapper::correctPose, this);
	setModeSrv = pn.advertiseService("set_mode", &Mapper::setMode, this);
	getModeSrv = pn.advertiseService("get_mode", &Mapper::getMode, this);
	getBoundedMapSrv = pn.advertiseService("get_bounded_map", &Mapper::getBoundedMap, this);

	// refreshing tf transform thread
	publishThread = boost::thread(boost::bind(&Mapper::publishLoop, this, tfRefreshPeriod));

}

Mapper::~Mapper()
{
  savePathToCsv("path.csv");

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
		unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scanMsgIn, &tfListener, odomFrame)));
		processCloud(move(cloud), scanMsgIn.header.frame_id, endScanTime, scanMsgIn.header.seq);
	}
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	if(localizing)
	{
		unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
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

// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
void Mapper::processCloud(unique_ptr<DP> newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{

	

	// if the future has completed, use the new map
	processNewMapIfAvailable(); // This call lock the tf publication
	cerr << "received new map" << endl;
	
	timer t;

  processingNewCloud = true;
	BoolSetter stopProcessingSetter(processingNewCloud, false);
	
	
	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

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

		cout << "Adding time" << endl;
		
	}

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
		publishLock.lock();
		TOdomToMap = PM::TransformationParameters::Identity(dimp1, dimp1);
		TOdomToMap(2,3) = mapElevation;
		publishLock.unlock();
	}

 
	// Fetch transformation from scanner to odom
	// Note: we don't need to wait for transform. It is already called in transformListenerToEigenMatrix()
	PM::TransformationParameters TOdomToScanner;
	try
	{
		TOdomToScanner = PointMatcher_ros::eigenMatrixToDim<float>(
				PointMatcher_ros::transformListenerToEigenMatrix<float>(
				tfListener,
				scannerFrame,
				odomFrame,
				stamp
			), dimp1);
	}
	catch(tf::ExtrapolationException e)
	{
		ROS_ERROR_STREAM("Extrapolation Exception. sf=" << scannerFrame << ", stamp = "<< stamp << " now = " << ros::Time::now() << " delta = " << ros::Time::now() - stamp << endl << e.what() );
		return;
	}


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
    //TODO TEST finish here....
    //Ticp = PM::TransformationParameters::Identity(4,4); 
    //END TEST
    Ticp = transformation->correctParameters(Ticp);

    // extract corrections
    PM::TransformationParameters Tdelta = Ticp * TscannerToMap.inverse();
     
		// ISER
		//{
    //  // remove roll and pitch
    //  Tdelta(2,0) = 0; 
    //  Tdelta(2,1) = 0; 
    //  Tdelta(2,2) = 1; 
    //  Tdelta(0,2) = 0; 
    //  Tdelta(1,2) = 0;
    //  Tdelta(2,3) = 0; //z
    //  Tdelta.block(0,0,3,3) = transformation->correctParameters(Tdelta.block(0,0,3,3));

    //  Ticp = Tdelta*TscannerToMap;

    //  ROS_DEBUG_STREAM("Ticp:\n" << Ticp);
		//}

		
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

    PM::TransformationParameters Terror = TscannerToMap.inverse() * Ticp;

    cerr << "Correcting translation error of " << Terror.block(0,3, 3,1).norm() << " m" << endl;

    // Add transformation to path
    ScanInfo info;
    info.pose = Ticp;
    info.error = Terror;
    info.timeStamp = stamp;

    path.push_back(info);

    //path.push_back(Ticp);
    //errors.push_back(Terror);

		// Publish tf
		if(publishMapTf == true)
		{
			tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TOdomToMap, mapFrame, odomFrame, stamp));
		}

		publishLock.unlock();
		processingNewCloud = false;
		
		ROS_DEBUG_STREAM("TOdomToMap:\n" << TOdomToMap);

		// Publish odometry
		if (odomPub.getNumSubscribers())
			odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(Tdelta, mapFrame, stamp));
	
    // TODO: check that, might be wrong....
		// Publish error on odometry
		if (odomErrorPub.getNumSubscribers())
			odomErrorPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TOdomToMap, mapFrame, stamp));

		// ***Debug:
    //debugPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(transformation->compute(*newPointCloud, Ticp), mapFrame, mapCreationTime));
		// ***


		// check if news points should be added to the map
		if (
			mapping &&
			((estimatedOverlap < maxOverlapToMerge) || (icp.getInternalMap().features.cols() < minMapPointCount)) &&
			(!mapBuildingInProgress)
    )
		{
			cout << "map Creation..." << endl;
			// make sure we process the last available map
			mapCreationTime = stamp;
			ROS_INFO("Adding new points to the map in background");
			mapBuildingTask = MapBuildingTask(boost::bind(&Mapper::updateMap, this, newPointCloud.release(), Ticp, true));
			mapBuildingFuture = mapBuildingTask.get_future();
			mapBuildingThread = boost::thread(boost::move(boost::ref(mapBuildingTask)));
			mapBuildingInProgress = true;
    }
	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	//Statistics about time and real-time capability
	int realTimeRatio = 100*t.elapsed() / (stamp.toSec()-lastPoinCloudTime.toSec());
	ROS_INFO_STREAM("[TIME] Total ICP took: " << t.elapsed() << " [s]");
	if(realTimeRatio < 80)
		ROS_INFO_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
	else
		ROS_WARN_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

	lastPoinCloudTime = stamp;
}

void Mapper::processNewMapIfAvailable()
{
	if (mapBuildingInProgress && mapBuildingFuture.has_value())
	{
		ROS_INFO_STREAM("New map available");
		setMap(mapBuildingFuture.get());
		mapBuildingInProgress = false;
	}
}

void Mapper::setMap(DP* newPointCloud)
{
	// delete old map
	if (mapPointCloud)
		delete mapPointCloud;
	
	// set new map
	mapPointCloud = newPointCloud;
	cerr << "copying map to ICP" << endl;
  //FIXME: this is taking the all map instead of the small part we need
	icp.setMap(*mapPointCloud); // This do a full copy...
	
	
	cerr << "publishing map" << endl;
	// Publish map point cloud
	// FIXME this crash when used without descriptor
	if (mapPub.getNumSubscribers())
		mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, mapFrame, mapCreationTime));
}

Mapper::DP* Mapper::updateMap(DP* newMap, const PM::TransformationParameters Ticp, bool updateExisting)
{
	timer t;
 
  // Prepare empty field if not existing
  // FIXME: this is only needed for the none overlaping part
	if(newMap->descriptorExists("probabilityStatic") == false)
	{
		//newMap->addDescriptor("probabilityStatic", PM::Matrix::Zero(1, newMap->features.cols()));
		newMap->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newMap->features.cols(), priorStatic));
	}
	
	if(newMap->descriptorExists("probabilityDynamic") == false)
	{
		//newMap->addDescriptor("probabilityDynamic", PM::Matrix::Zero(1, newMap->features.cols()));
		newMap->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newMap->features.cols(), priorDyn));
	}
	
	if(newMap->descriptorExists("debug") == false)
	{
		newMap->addDescriptor("debug", PM::Matrix::Zero(1, newMap->features.cols()));
	}

	if (!updateExisting)
	{
		// FIXME: correct that, ugly
		cout << "Jumping map creation" << endl;
		*newMap = transformation->compute(*newMap, Ticp); 
		mapPostFilters.apply(*newMap);
		return newMap;
	}

	 // FIXME: only for debug
	mapPointCloud->getDescriptorViewByName("debug") = PM::Matrix::Zero(1, mapPointCloud->features.cols());


	
	const int newMapPts(newMap->features.cols());
	const int mapPtsCount(mapPointCloud->features.cols());
	
	// Build a range image of the reading point cloud (local coordinates)
	PM::Matrix radius_newMap = newMap->features.topRows(3).colwise().norm();

	PM::Matrix angles_newMap(2, newMapPts); // 0=inclination, 1=azimuth

	// No atan in Eigen, so we are for to loop through it...
	for(int i=0; i<newMapPts; i++)
	{
		const float ratio = newMap->features(2,i)/radius_newMap(0,i);

		angles_newMap(0,i) = acos(ratio);
		angles_newMap(1,i) = atan2(newMap->features(1,i), newMap->features(0,i));
	}

	std::shared_ptr<NNS> kdtree;
	kdtree.reset( NNS::create(angles_newMap));

  //-------------- Global map ------------------------------
	// Transform the global map in local coordinates
	DP mapLocal = transformation->compute(*mapPointCloud, Ticp.inverse());

  // ROI: Region of Interest
  // We reduce the global map to the minimum for the processing

	//const float sensorMaxRange = 80.0; // ICRA
	
  PM::Matrix globalId(1, mapPtsCount); 

	int ROIpts = 0;
	int notROIpts = 0;

  // Split mapLocal
	DP mapLocalROI(mapLocal.createSimilarEmpty());
	for (int i = 0; i < mapPtsCount; i++)
	{
    // Copy the points of the ROI in a new map
		if (mapLocal.features.col(i).head(3).norm() < sensorMaxRange)
		{
			mapLocalROI.setColFrom(ROIpts, mapLocal, i);
			globalId(0,ROIpts) = i;
			ROIpts++;
		}
    else // Remove the points of the ROI from the global map
    {
			mapLocal.setColFrom(notROIpts, mapLocal, i);
			notROIpts++;
    }
	}

	mapLocalROI.conservativeResize(ROIpts);
	mapLocal.conservativeResize(notROIpts);
	

  // Convert the map in spherical coordinates
	PM::Matrix radius_map = mapLocalROI.features.topRows(3).colwise().norm();
	PM::Matrix angles_map(2, ROIpts); // 0=inclination, 1=azimuth

	// No atan in Eigen, so we are looping through it...
  // TODO: check for: A.binaryExpr(B, std::ptr_fun(atan2))
	for(int i=0; i < ROIpts; i++)
	{
		const float ratio = mapLocalROI.features(2,i)/radius_map(0,i);
		//if(ratio < -1 || ratio > 1)
			//cout << "Error angle!" << endl;

		angles_map(0,i) = acos(ratio);

		angles_map(1,i) = atan2(mapLocalROI.features(1,i), mapLocalROI.features(0,i));
	}

  // Prepare access to descriptors
	DP::View viewOn_Msec_overlap = newMap->getDescriptorViewByName("stamps_Msec");
	DP::View viewOn_sec_overlap = newMap->getDescriptorViewByName("stamps_sec");
	DP::View viewOn_nsec_overlap = newMap->getDescriptorViewByName("stamps_nsec");

	DP::View viewOnProbabilityStatic = mapLocalROI.getDescriptorViewByName("probabilityStatic");
	DP::View viewOnProbabilityDynamic = mapLocalROI.getDescriptorViewByName("probabilityDynamic");
	DP::View viewDebug = mapLocalROI.getDescriptorViewByName("debug");
	
	DP::View viewOn_normals_map = mapLocalROI.getDescriptorViewByName("normals");
	DP::View viewOn_Msec_map = mapLocalROI.getDescriptorViewByName("stamps_Msec");
	DP::View viewOn_sec_map = mapLocalROI.getDescriptorViewByName("stamps_sec");
	DP::View viewOn_nsec_map = mapLocalROI.getDescriptorViewByName("stamps_nsec");
	
	// Search for the nearest point in newMap
	Matches::Dists dists(1, ROIpts);
	Matches::Ids ids(1, ROIpts);
	
	kdtree->knn(angles_map, ids, dists, 1, 0, NNS::ALLOW_SELF_MATCH, maxAngle);


  // update probability of being dynamic for all points in ROI
	for(int i=0; i < ROIpts; i++)
	{
    const int mapId = i;
    viewDebug(0,mapId) = 1; //FIXME: debug

		if(dists(i) != numeric_limits<float>::infinity())
		{
			const int readId = ids(0,i);
			//const int mapId = globalId(0,i);
						
			// in local coordinates
			const Eigen::Vector3f readPt = newMap->features.col(readId).head(3);
			const Eigen::Vector3f mapPt = mapLocalROI.features.col(i).head(3);
			const Eigen::Vector3f mapPt_n = mapPt.normalized();
			const float delta = (readPt - mapPt).norm();
			const float d_max = eps_a * readPt.norm();

      const Eigen::Vector3f normal_map = viewOn_normals_map.col(mapId);
			
      // Weight for dynamic elements
			const float w_v = eps + (1 - eps)*fabs(normal_map.dot(mapPt_n));
			const float w_d1 =  eps + (1 - eps)*(1 - sqrt(dists(i))/maxAngle);
			
			const float offset = delta - eps_d;
			float w_d2 = 1;
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
					w_p2 = eps + (1 - eps)*(1 - offset/d_max);
				}
			}


			// We don't update point behind the reading
			if((readPt.norm() + eps_d + d_max) >= mapPt.norm())
			{
				const float lastDyn = viewOnProbabilityDynamic(0,mapId);
				const float lastStatic = viewOnProbabilityStatic(0, mapId);

				const float c1 = (1 - (w_v*(1 - w_d1)));
				const float c2 = w_v*(1 - w_d1);
			
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
				
				//viewDebug(0,mapId) =viewOnProbabilityDynamic(0, mapId);
				//viewDebug(0,mapId) = w_d2;

				// Refresh time
				viewOn_Msec_map(0,mapId) = viewOn_Msec_overlap(0,readId);	
				viewOn_sec_map(0,mapId) = viewOn_sec_overlap(0,readId);	
				viewOn_nsec_map(0,mapId) = viewOn_nsec_overlap(0,readId);	
			}
		}
	}

	// Compute density
  const int mapLocalROIPts = mapLocalROI.features.cols();
	cout << "build first kdtree with " << mapLocalROIPts << endl;
	// build and populate NNS
	std::shared_ptr<NNS> kdtree2;
	kdtree2.reset( NNS::create(mapLocalROI.features, mapLocalROI.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
	
	PM::Matches matches_overlap(
		Matches::Dists(1, newMapPts),
		Matches::Ids(1, newMapPts)
	);
	
	kdtree2->knn(newMap->features, matches_overlap.ids, matches_overlap.dists, 1, 0, NNS::ALLOW_SELF_MATCH, maxDistNewPoint);
	
  //-------------- New point cloud ------------------------------

	DP newMapOverlap(newMap->createSimilarEmpty());// Not used for now
  PM::Matrix minDist = PM::Matrix::Constant(1,mapLocalROIPts, std::numeric_limits<float>::max());
	
  // Reduce newMap to its none overlapping part
	int ptsOut = 0;
	int ptsIn = 0;

  bool hasSensorNoise = mapLocalROI.descriptorExists("simpleSensorNoise") && newMap->descriptorExists("simpleSensorNoise");
  if(hasSensorNoise) // Split and update point with lower noise
  {
    DP::View viewOn_noise_mapLocal = mapLocalROI.getDescriptorViewByName("simpleSensorNoise");
    DP::View viewOn_noise_newMap = newMap->getDescriptorViewByName("simpleSensorNoise");

    for (int i = 0; i < newMapPts; ++i)
    {
      const int localMapId = matches_overlap.ids(i);

      if (matches_overlap.dists(i) == numeric_limits<float>::infinity())
      {
        newMap->setColFrom(ptsOut, *newMap, i);
        ptsOut++;
      }
      else // Overlapping points
      {
        // Update point with lower sensor noise
        if(viewOn_noise_newMap(0,i) < viewOn_noise_mapLocal(0,localMapId))
        {
          if(matches_overlap.dists(i) < minDist(localMapId))
          {
            minDist(localMapId) = matches_overlap.dists(i);
            const float debug = viewDebug(0, localMapId);
            const float probDyn = viewOnProbabilityDynamic(0, localMapId);
            const float probStatic = viewOnProbabilityStatic(0, localMapId);

            mapLocalROI.setColFrom(localMapId, *newMap, i); //TODO: check if descriptor follow...
            viewDebug(0, localMapId) = 2;  
            viewOnProbabilityDynamic(0, localMapId) = probDyn;
            viewOnProbabilityStatic(0, localMapId) = probStatic;
          }
        }

        newMapOverlap.setColFrom(ptsIn, *newMap, i);
        ptsIn++;
      }
    }
  }
  else // Only split newMap
  {
    for (int i = 0; i < newMapPts; ++i)
    {
      if (matches_overlap.dists(i) == numeric_limits<float>::infinity())
      {
        newMap->setColFrom(ptsOut, *newMap, i);
        ptsOut++;
      }
      else // Overlapping points
      {
        newMapOverlap.setColFrom(ptsIn, *newMap, i);
        ptsIn++;
      }
    }
  }

	// shrink the newMap to the new information
	newMap->conservativeResize(ptsOut);
	newMapOverlap.conservativeResize(ptsIn);

	cout << "ptsOut=" << ptsOut << ", ptsIn=" << ptsIn << endl;

	// Publish debug
	//if (debugPub.getNumSubscribers())
	//{
  //    debugPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*newMap, mapFrame, mapCreationTime));
	//}

  //no_overlap.addDescriptor("debug", PM::Matrix::Zero(1, no_overlap.features.cols()));
	


	// Add the ROI to the none overlapping part
	newMap->concatenate(mapLocalROI);
  
  // Apply the filters only on the ROI
  mapPostFilters.apply(*newMap);

  // Add the rest of the map
	newMap->concatenate(mapLocal);
	
  // Transform the map back to global coordinates
	*newMap = transformation->compute(*newMap, Ticp);

	cout << "... end map creation" << endl;
	ROS_INFO_STREAM("[TIME] New map available (" << newMap->features.cols() << " pts), update took " << t.elapsed() << " [s]");
	
	return newMap;
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
	if(publishPeriod == 0) //FIXME: just don't start the thread if 0
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

  // TODO: test for TREX video, put that back
	//if(processingNewCloud == false && publishMapTf == true)
	if(publishMapTf == true)
	{
		publishLock.lock();
		// Note: we use now as timestamp to refresh the tf and avoid other buffer to be empty
		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TOdomToMap, mapFrame, odomFrame, ros::Time::now()));
		publishLock.unlock();
	}
  else
  {
    //cerr << "NOT PUBLISHING: " << processingNewCloud << endl;
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

  int lastindex = req.filename.data.find_last_of("."); 
  string rawname = req.filename.data.substr(0, lastindex);
	
  try
	{
    savePathToCsv(rawname+"_path.csv");
		mapPointCloud->save(req.filename.data);
	}
	catch (const std::runtime_error& e)
	{
		ROS_ERROR_STREAM("Unable to save: " << e.what());
		return false;
	}
	
	ROS_INFO_STREAM("Map saved at " <<  req.filename.data << " with " << mapPointCloud->features.cols() << " points.");
	return true;
}

bool Mapper::loadMap(ethzasl_icp_mapper::LoadMap::Request &req, ethzasl_icp_mapper::LoadMap::Response &res)
{
	waitForMapBuildingCompleted();
	
	DP* cloud(new DP(DP::load(req.filename.data)));

	const int dim = cloud->features.rows();
	const int nbPts = cloud->features.cols();
	ROS_INFO_STREAM("Loading " << dim-1 << "D point cloud (" << req.filename.data << ") with " << nbPts << " points.");

	publishLock.lock();
	TOdomToMap = PM::TransformationParameters::Identity(dim,dim);
	
	//ISER
	//TOdomToMap(2,3) = mapElevation;

	publishLock.unlock();

  // Prepare descriptor if not existing
  if(cloud->descriptorExists("probabilityStatic") == false)
	{
		cloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, cloud->features.cols(), priorStatic));
	}
	
	if(cloud->descriptorExists("probabilityDynamic") == false)
	{
		cloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, cloud->features.cols(), priorDyn));
	}
	
	if(cloud->descriptorExists("debug") == false)
	{
		cloud->addDescriptor("debug", PM::Matrix::Zero(1, cloud->features.cols()));
	}

	setMap(cloud);
	
	return true;
}

bool Mapper::reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	waitForMapBuildingCompleted();
	
	// note: no need for locking as we do ros::spin(), to update if we go for multi-threading
	publishLock.lock();
	TOdomToMap = PM::TransformationParameters::Identity(4,4);
	publishLock.unlock();

	icp.clearMap();
	
	return true;
}

bool Mapper::correctPose(ethzasl_icp_mapper::CorrectPose::Request &req, ethzasl_icp_mapper::CorrectPose::Response &res)
{
	publishLock.lock();
	TOdomToMap = PointMatcher_ros::odomMsgToEigenMatrix<float>(req.odom);
	
	//ISER
	//{
	//// remove roll and pitch
	//TOdomToMap(2,0) = 0; 
	//TOdomToMap(2,1) = 0; 
	//TOdomToMap(2,2) = 1; 
	//TOdomToMap(0,2) = 0; 
	//TOdomToMap(1,2) = 0;
	//TOdomToMap(2,3) = mapElevation; //z
	//}

	tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TOdomToMap, mapFrame, odomFrame, ros::Time::now()));
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

	cerr << "min [" << min_x << ", " << min_y << ", " << min_z << "] " << endl;
	cerr << "max [" << max_x << ", " << max_y << ", " << max_z << "] " << endl;



	tf::StampedTransform stampedTr;
	
	Eigen::Affine3d eigenTr;
	tf::poseMsgToEigen(req.mapCenter, eigenTr);
	Eigen::MatrixXf T = eigenTr.matrix().inverse().cast<float>();
	//const Eigen::MatrixXf T = eigenTr.matrix().cast<float>();

	cerr << "T:" << endl << T << endl;
	T = transformation->correctParameters(T);

		
	// FIXME: do we need a mutex here?
	const DP centeredPointCloud = transformation->compute(*mapPointCloud, T); 
	DP cutPointCloud = centeredPointCloud.createSimilarEmpty();

	cerr << centeredPointCloud.features.topLeftCorner(3, 10) << endl;
	cerr << T << endl;

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

	cerr << "Extract " << newPtCount << " points from the map" << endl;
	
	cutPointCloud.conservativeResize(newPtCount);
	cutPointCloud = transformation->compute(cutPointCloud, T.inverse()); 

	
	// Send the resulting point cloud in ROS format
	res.boundedMap = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(cutPointCloud, mapFrame, ros::Time::now());
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
