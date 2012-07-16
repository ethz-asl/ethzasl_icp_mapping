#include <fstream>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include "aliases.h"
#include "get_params_from_server.h"
#include "ros_logger.h"


using namespace std;
using namespace PointMatcherSupport;

class PublishVTK
{
	ros::NodeHandle& n;
	const DP cloud;
	const string mapFrame;
	ros::Publisher cloudPub;

public:
	PublishVTK(ros::NodeHandle& n, const std::string& fileName);
	void publish();
	void run();
};

PublishVTK::PublishVTK(ros::NodeHandle& n, const std::string& fileName):
	n(n),
	cloud(PM::loadVTK(fileName)),
	mapFrame(getParam<string>("mapFrameId", "/map"))
{
	// ROS initialization
	cloudPub = n.advertise<sensor_msgs::PointCloud2>(
		getParam<string>("cloudTopic", "/point_cloud"), 1
	);
}

void PublishVTK::publish()
{
	if (cloudPub.getNumSubscribers())
		cloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(cloud, mapFrame, ros::Time::now()));
}

void PublishVTK::run()
{
	ros::Rate r(1);
	while (ros::ok())
	{
		publish();
		r.sleep();
	}
}

// Main function supporting the ExportVtk class
int main(int argc, char **argv)
{
	if (argc < 2)
	{
		cerr << "Usage " << argv[0] << " filename" << endl;
		return 1;
	}
	
	ros::init(argc, argv, "VtkToPointCloud_node");
	ros::NodeHandle n;
	PublishVTK pub(n, argv[1]);
	pub.run();
	
	return 0;
}
