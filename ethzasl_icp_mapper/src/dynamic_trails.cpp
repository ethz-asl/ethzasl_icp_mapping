#include <fstream>

#include <boost/version.hpp>

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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

using namespace std;
using namespace PointMatcherSupport;
using namespace visualization_msgs;

class DynamicTrails
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::TransformationParameters TP;
	typedef PM::Matches Matches;
	
	typedef typename Nabo::NearestNeighbourSearch<float> NNS;
	typedef typename NNS::SearchType NNSearchType;

	ros::NodeHandle& n;
	ros::NodeHandle& pn;

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync;

	ros::Subscriber scanSub;
	ros::Subscriber cloudSub;
	ros::Subscriber mapSub;
	ros::Publisher lastPub;
	ros::Publisher markerPub;

	PM::DataPointsFilters inputFilters;
	unique_ptr<PM::Transformation> transformation;
	PM::DataPoints lastPointCloud;
	PM::DataPoints globalMap;
	
	std::shared_ptr<NNS> globalNNS;

	boost::mutex usingGlobalMap;
	
	// Parameters
	//...
	
	// tf
	tf::TransformListener tfListener;

	boost::mutex publishLock;
	// Parameters
	string mapFrame;
	int trailCount;
	
public:
	DynamicTrails(ros::NodeHandle& n, ros::NodeHandle& pn);
	~DynamicTrails();
	
protected:
	void gotCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn, const nav_msgs::OdometryConstPtr& odom);
	void gotMap(const sensor_msgs::PointCloud2& cloudMsgIn);
	void processCloud(DP cloud, const TP TScannerToMap);
	
	};

DynamicTrails::DynamicTrails(ros::NodeHandle& n, ros::NodeHandle& pn):
	n(n),
	pn(pn),
	cloud_sub(n, "cloud_in", 1),
	odom_sub(n, "icp_odom", 1),
	sync(MySyncPolicy(10), cloud_sub, odom_sub),
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
  tfListener(ros::Duration(30)),
	mapFrame(getParam<string>("map_frame", "map")),
	trailCount(0)
{
	
	// load configs
	string configFileName;

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

	
	// topics and services initialization
	sync.registerCallback(boost::bind(&DynamicTrails::gotCloud, this, _1, _2));

	mapSub = n.subscribe("point_map", 1, &DynamicTrails::gotMap, this);
	lastPub = n.advertise<sensor_msgs::PointCloud2>("extracted_dynamic_points", 2, true);
	markerPub = n.advertise<visualization_msgs::Marker>("dynamic_trail", 1);
		
}

DynamicTrails::~DynamicTrails()
{
	// save point cloud
	//...
	
}

// Callback
void DynamicTrails::gotMap(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	cout << "Map received" << endl;
	usingGlobalMap.lock();
	// Convert pointcloud2 to DataPoint
	globalMap = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn);
	
	// Build a kd-tree
	globalNNS.reset( NNS::create(globalMap.features, globalMap.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
	usingGlobalMap.unlock();
}

void DynamicTrails::gotCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn, const nav_msgs::OdometryConstPtr& odom)
{
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)

	// Convert pointcloud2 to DataPoint
	DP cloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*cloudMsgIn));
	// Convert odometry msg to TransformationParameters
	TP TScannerToMap = 	PointMatcher_ros::odomMsgToEigenMatrix<float>(*odom);
	processCloud(cloud, TScannerToMap);

}

// Point cloud processing
void DynamicTrails::processCloud(DP newPointCloud, const TP TScannerToMap)
{

	ROS_INFO("Processing new point cloud");
	
	timer t; // Print how long take the algo
		
	// Apply filters to incoming cloud, in scanner coordinates
	inputFilters.apply(newPointCloud);
		
	ROS_INFO_STREAM("Input filters took " << t.elapsed() << " [s]");
	
	// Correct new points using ICP result and move them in their own frame
	newPointCloud = transformation->compute(newPointCloud, TScannerToMap); 

	const int newPtsCount(newPointCloud.features.cols());

	Matches::Dists dists_new_full(1, newPtsCount);
	Matches::Ids ids_new_full(1, newPtsCount);
	
	usingGlobalMap.lock();
	const float maxGlobalDist = 0.5;
	globalNNS->knn(newPointCloud.features, ids_new_full, dists_new_full, 1, 0, maxGlobalDist);

	if(globalMap.descriptorExists("observedTime") == false ||
			globalMap.descriptorExists("unobservedTime") == false ||
			globalMap.descriptorExists("dynamic_ratio") == false
		)
	{
		ROS_INFO_STREAM("Map descriptors not found");
		usingGlobalMap.unlock();
		return;
	}

	DP::View viewOnDynamicRatio = globalMap.getDescriptorViewByName("dynamic_ratio");
	DP::View viewOnObservedTime = globalMap.getDescriptorViewByName("observedTime");
	DP::View viewOnUnobservedTime = globalMap.getDescriptorViewByName("unobservedTime");

	DP dynamicPoints(newPointCloud.createSimilarEmpty());
	int dynamicPtsCount=0;
	for(int i=0; i<newPtsCount; i++)
	{
		const int mapId = ids_new_full(i);
		const float sumObs = viewOnUnobservedTime(0,mapId) + viewOnObservedTime(0,mapId);

		// FIXME: this is a parameter
		const float minDynamicRatio = 0.5;
		if(dists_new_full(i) != numeric_limits<float>::infinity())
		{
			if(sumObs == 0 || viewOnDynamicRatio(0, mapId) > minDynamicRatio)
			//if(viewOnDynamicRatio(0, mapId) > minDynamicRatio)
			{
				dynamicPoints.setColFrom(dynamicPtsCount, newPointCloud, i);
				dynamicPtsCount++;
			}
		}
	}

	dynamicPoints.conservativeResize(dynamicPtsCount);
	usingGlobalMap.unlock();

	if(lastPointCloud.features.cols() == 0)
	{
		lastPointCloud = dynamicPoints;
		lastPointCloud.addDescriptor("velocity_mag", PM::Matrix::Zero(1,dynamicPtsCount));
		lastPointCloud.addDescriptor("velocity_dir", PM::Matrix::Zero(3,dynamicPtsCount));


		cout << "skipping the first point cloud" << endl;
		return;
	}

	const int lastPtsCount = lastPointCloud.features.cols();
	cout << "Last point size: " <<  lastPtsCount << ", new point size: " << newPtsCount << ", dynamicPtsCount: " << dynamicPtsCount << endl;

	string reason;


	// Check dimension
	if (dynamicPoints.features.rows() != lastPointCloud.features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << dynamicPoints.features.rows()-1 << " while lastPointCloud is " << lastPointCloud.features.rows()-1);
		return;
	}

	
	DP::View viewOn_normals_new = dynamicPoints.getDescriptorViewByName("normals");
	DP::View viewOn_normals_last = lastPointCloud.getDescriptorViewByName("normals");

	PM::Matrix lastEuclNormal(6, lastPtsCount);

	lastEuclNormal.topRows(3) = lastPointCloud.features.topRows(3);
	lastEuclNormal.bottomRows(3) = viewOn_normals_last*1.5;
	
	PM::Matrix newEuclNormal(6, dynamicPtsCount);
	newEuclNormal.topRows(3) = dynamicPoints.features.topRows(3);
	newEuclNormal.bottomRows(3) = viewOn_normals_new*1.5;
	

	
	std::shared_ptr<NNS> featureNNS;
	// Compute velocities from new to last
	featureNNS.reset( NNS::create(lastEuclNormal, 6, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
	//featureNNS.reset( NNS::create(lastPointCloud.features, 3, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));

	const int knn_new = 1;
	Matches::Dists dists_new(knn_new, dynamicPtsCount);
	Matches::Ids ids_new(knn_new, dynamicPtsCount);
	

	featureNNS->knn(newEuclNormal, ids_new, dists_new, knn_new, 0);
	//featureNNS->knn(dynamicPoints.features, ids_new, dists_new, 1, 0);
	
	// Compute velocities from last to new
	featureNNS.reset( NNS::create(newEuclNormal, 6, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
	//featureNNS.reset( NNS::create(dynamicPoints.features, 3, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));

	Matches::Dists dists_last(1, lastPtsCount);
	Matches::Ids ids_last(1, lastPtsCount);
	

	featureNNS->knn(lastEuclNormal, ids_last, dists_last, 1, 0);
	//featureNNS->knn(lastPointCloud.features, ids_last, dists_last, 1, 0);

	
	cout << "Finish search" << endl;


	// Markers
	visualization_msgs::Marker marker;

	marker.header.frame_id = mapFrame;
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	//marker.id = 0;
	marker.id = trailCount;
	trailCount++;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	
	//marker.pose.position.x = 0;
	//marker.pose.position.y = 0;
	//marker.pose.position.z = 0;
	//marker.pose.orientation.x = 0.0;
	//marker.pose.orientation.y = 0.0;
	//marker.pose.orientation.z = 0.0;
	//marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.05;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.10;

	marker.lifetime = ros::Duration();

	DP::View viewOn_Msec_new = dynamicPoints.getDescriptorViewByName("stamps_Msec");
	DP::View viewOn_sec_new = dynamicPoints.getDescriptorViewByName("stamps_sec");
	DP::View viewOn_nsec_new = dynamicPoints.getDescriptorViewByName("stamps_nsec");
	
	DP::View viewOn_Msec_last = lastPointCloud.getDescriptorViewByName("stamps_Msec");
	DP::View viewOn_sec_last = lastPointCloud.getDescriptorViewByName("stamps_sec");
	DP::View viewOn_nsec_last = lastPointCloud.getDescriptorViewByName("stamps_nsec");
	

	PM::Matrix orientations(3, dynamicPtsCount);
	PM::Matrix velocities(1, dynamicPtsCount);
	PM::Matrix deltaTimes(1, dynamicPtsCount);
	
	for(int i=0; i<dynamicPtsCount; i++)
	{
		for(int k=0; k<knn_new; k++)
		{
			const int lastId = ids_new(k, i);
			const Eigen::Vector3f newP = dynamicPoints.features.col(i).head(3);
			Eigen::Vector3f lastP = lastPointCloud.features.col(lastId).head(3);
			Eigen::Vector3f dir = (newP - lastP);
			Eigen::Vector3f dp = dir.normalized();
			float distance = dir.norm();

			// Time
			const double timeLast = viewOn_Msec_last(0,lastId)*10e5 + viewOn_sec_last(0,lastId) + viewOn_nsec_last(0,lastId)*10e-10;
			const double timeNew = viewOn_Msec_new(0,i)*10e5 + viewOn_sec_new(0,i) + viewOn_nsec_new(0,i)*10e-10;
			const double dt = timeNew - timeLast;

			int NN = 1;

			for(int j=0; j<lastPtsCount; j++)
			{
				const int newId = ids_last(j);
				if(i == newId)
				{
					lastP = lastPointCloud.features.col(j).head(3);
					dir = (newP - lastP);
					dp += dir.normalized();
					distance += dir.norm();
					NN++;
				}
			}
			dp = dp/NN;
			dp.normalize();
			orientations.col(i) = dp;

			distance = distance/NN;
			velocities(i) = distance/dt;
			deltaTimes(i) = dt;

			//std::cout.setf( std::ios::fixed);
			//cout << timeNew << ", " << timeLast << endl;
			//cout << distance << ", " << deltaTime << ", " << distance/deltaTime << endl;
		}	
	}
	
	
	// Smoothing vector based on surrounding vectors
	// FIXME: those are parameters
	const float maxDynamicDist = 0.5;
	const int knn = 20;
	Matches::Dists dists_smooth(knn, dynamicPtsCount);
	Matches::Ids ids_smooth(knn, dynamicPtsCount);


	featureNNS->knn(newEuclNormal, ids_smooth, dists_smooth, knn, 0, maxDynamicDist);
	//featureNNS->knn(dynamicPoints.features, ids_smooth, dists_smooth, knn, 0, maxDynamicDist);


	DP::View viewOnVelocity_mag_last = lastPointCloud.getDescriptorViewByName("velocity_mag");
	DP::View viewOnVelocity_dir_last = lastPointCloud.getDescriptorViewByName("velocity_dir");

	DP dynamicPointsFiltered(dynamicPoints.createSimilarEmpty());
	int dynamicFilteredPtsCount = 0;
	for(int i=0; i<dynamicPtsCount; i++)
	{
		const Eigen::Vector3f newP = dynamicPoints.features.col(i).head(3);
		Eigen::Vector3f dp = orientations.col(i);
		float vel = velocities(i);
		int NN = 1;

		for(int k=0; k<knn; k++)
		{
			if(dists_smooth(k,i) != numeric_limits<float>::infinity())
			{
				const int otherId = ids_smooth(k,i);
				dp += orientations.col(otherId);
				vel += velocities(otherId);
				NN++;
			}
			
		}

		dp = dp/NN;
		dp.normalize();

		vel = vel/NN;


		const Eigen::Vector3f normal = viewOn_normals_new.col(i);
		
		// FIXME: those are parameters
		const float maxVel = 20.0;
		const float minVel = 0.2;

		// Project based on last velocity
		const int lastId = ids_new(i);
		const float dt = deltaTimes(i);
		const float vel_last = viewOnVelocity_mag_last(0,lastId);
		const Eigen::Vector3f vel_dir_last = viewOnVelocity_dir_last.col(lastId);
		const Eigen::Vector3f lastP = lastPointCloud.features.col(lastId).head(3);
		const Eigen::Vector3f lastP_projected = lastP + vel_dir_last*vel_last*dt;
		const Eigen::Vector3f newP_projected = newP - dp*vel*dt;

		const float currentDist = (lastP - newP).norm();
		const float lastDist = (lastP_projected - newP).norm();
		const float newDist = (newP_projected - lastP).norm();

		float t_lastProjection = 1.0 - min(1.0f, lastDist/currentDist);
		if(vel_last ==0)
			t_lastProjection = 0.9;

		const float t_newProjection = 1.0 - min(1.0f, newDist/currentDist);
		const float t_movingZone = NN/(knn + 1);
		//const float t_normalAngle = 1- acos(dp.dot(normal))/M_PI;
		const float t_normalAngle = abs(dp.dot(normal));
		const float t_vel = (minVel < vel && vel < maxVel);
		const float t_total = 
			t_lastProjection * 
			t_newProjection * 
			t_movingZone * 
			pow(t_normalAngle,2) * 
			t_vel;


		vel = vel*t_total;
		vel = vel*(vel > minVel);
		//cout << vel << ", " << t_total << endl;
		orientations.col(i) = dp;
		velocities(i) = vel;
		//cout << t_lastProjection << ", " << t_newProjection  << ", " << t_movingZone << ", " << t_normalAngle << ", " << t_vel << endl;
		//cout << t_total << endl;

		if(vel)
		{
			geometry_msgs::Point p;
			p.x = newP(0);
			p.y = newP(1);
			p.z = newP(2);
			marker.points.push_back(p);
			
			p.x = newP(0) + dp(0)*vel;
			p.y = newP(1) + dp(1)*vel;
			p.z = newP(2) + dp(2)*vel;
			marker.points.push_back(p);
			dynamicPointsFiltered.setColFrom(dynamicFilteredPtsCount, dynamicPoints, i);
			dynamicFilteredPtsCount++;
		}
	}
	dynamicPointsFiltered.conservativeResize(dynamicFilteredPtsCount);
	
	markerPub.publish(marker);

	dynamicPoints.addDescriptor("velocity_mag", velocities);
	dynamicPoints.addDescriptor("velocity_dir", orientations);


	lastPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(dynamicPoints, mapFrame, ros::Time::now()));


	lastPointCloud = dynamicPoints;


	ROS_INFO_STREAM("Total trail computation: " << t.elapsed() << " [s]");

	//ros::Rate r(2);
	//r.sleep();
}





// Main function supporting the DynamicTrails class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_maintainer");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	DynamicTrails trails(n, pn);

	ros::Rate r(50);
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
