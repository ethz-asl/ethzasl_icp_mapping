#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "tf/transform_listener.h"
#include <vector>

using namespace std;

tf::TransformListener *listenerPtr(0);

struct Entry
{
	sensor_msgs::PointCloud::_points_type points;
	tf::Transform transform;
	
	Entry(const sensor_msgs::PointCloud& cloud, const tf::Transform& transform):
		points(cloud.points), transform(transform) {}
};
typedef vector<Entry> Entries;

Entries entries;
int maxCloudCount(0);

void pointCloudCallback(const sensor_msgs::PointCloud2& cloudMsg)
{
	// convert cloud
	sensor_msgs::PointCloud output;
	sensor_msgs::convertPointCloud2ToPointCloud(cloudMsg, output);
	
	// get transform
	tf::StampedTransform transform;
	try
	{
		listenerPtr->lookupTransform("/vicon_vehicle_20", "/ned",  ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	
	// store cloud along transform
	//ROS_WARN_STREAM("Added cloud " << entries.size());
	entries.push_back(Entry(output, transform));
	if (int(entries.size()) >= maxCloudCount)
		ros::shutdown();
}

void dumpData()
{
	cout << "# ETHZ ASL kinect exp data set\n";
	cout << entries.size() << "\n";
	for (size_t i = 0; i < entries.size(); ++i)
	{
		const Entry& entry(entries[i]);
		cout << "# entry " << i << "\n";
		const tf::Vector3& t(entry.transform.getOrigin());
		const tf::Quaternion r(entry.transform.getRotation());
		cout << t.x() << " " << t.y() << " " << t.z() << " " << r.x() << " " << r.y() << " " << r.z() << " " << r.w() << "\n";
		cout << entry.points.size() << "\n";
		for (size_t j = 0; j < entry.points.size(); ++j)
		{
			const geometry_msgs::Point32& p(entry.points[j]);
			cout << p.x << " " << p.y << " " << p.z << "\n";
		}
	}
}

int main(int argc, char **argv)
{
	if (argc != 2)
	{
		cerr << "Usage: " << argv[0] << " cloud_count\n";
		return 1;
	}
	maxCloudCount = atoi(argv[1]);
	
	ros::init(argc, argv, "kinect_exp_logger");
	ros::NodeHandle n;
	tf::TransformListener listener;
	listenerPtr = &listener;
	
	ros::Subscriber sub = n.subscribe("/camera/depth/points2", 10, pointCloudCallback);

	ros::spin();
	
	dumpData();
	
	return 0;
}
