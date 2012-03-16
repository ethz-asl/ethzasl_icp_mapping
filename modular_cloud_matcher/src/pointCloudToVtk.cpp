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

class ExportVtk
{
	ros::NodeHandle& n;
	
	ros::Subscriber cloudSub;
	string cloudTopic;
	string mapFrame;

	tf::TransformListener tf_listener;

public:
	ExportVtk(ros::NodeHandle& n);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsg);
};

ExportVtk::ExportVtk(ros::NodeHandle& n):
	n(n)
{
	// ROS initialization
	cloudTopic = getParam<string>("cloudTopic", "/static_point_cloud");
	cloudSub = n.subscribe(cloudTopic, 100, &ExportVtk::gotCloud, this);
	
	// Parameters for 3D map
	mapFrame= getParam<string>("mapFrameId", "/map");
}


void ExportVtk::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
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
	else
	{
		ROS_INFO("Saving cloud");
	}
	
	stringstream nameStream;
	nameStream << "." << cloudTopic << "_" << cloudMsg.header.seq;
	PM::saveVTK(newPointCloud, nameStream.str());

}

// Main function supporting the ExportVtk class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "pointCloudToVtk_node");
	ros::NodeHandle n;
	ExportVtk exporter(n);
	ros::spin();
	
	return 0;
}
