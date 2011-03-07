#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"
#include <vector>
#include <Eigen/Eigen>

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

Eigen::Vector3d trans_corr(Eigen::Vector3d(0,0,0));
Eigen::eigen2_Quaterniond rot_corr(Eigen::eigen2_Quaterniond::Identity());

void pointCloudCallback(const sensor_msgs::PointCloud2& cloudMsg)
{
	// convert cloud
	sensor_msgs::PointCloud output;
	sensor_msgs::convertPointCloud2ToPointCloud(cloudMsg, output);
	
	// get transform
	tf::StampedTransform transform;
	try
	{
		listenerPtr->lookupTransform( "/ned", "/vicon_vehicle_20",  ros::Time(0), transform);
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
	const Eigen::eigen2_Transform3d t_corr = Eigen::eigen2_Translation3d(trans_corr) * rot_corr;
	cout << "# ETHZ ASL kinect exp data set\n";
	cout << entries.size() << "\n";
	for (size_t i = 0; i < entries.size(); ++i)
	{
		const Entry& entry(entries[i]);
		cout << "# entry " << i << "\n";
		
		const tf::Vector3& t(entry.transform.getOrigin());
		const tf::Quaternion r(entry.transform.getRotation());
		Eigen::eigen2_Quaterniond t_local_rot;
		tf::RotationTFToEigen(r, t_local_rot);
		const Eigen::eigen2_Transform3d t_local =
			 Eigen::eigen2_Translation3d(Eigen::Vector3d(t)) * t_local_rot;
		const Eigen::eigen2_Transform3d t_global = Eigen::eigen2_Transform3d(t_corr.inverse()) * t_local;
		const Eigen::Matrix3d t_global_m(t_global.matrix().topLeftCorner(3,3));
		const Eigen::eigen2_Quaterniond t_g_rot = Eigen::eigen2_Quaterniond(t_global_m);
		const Eigen::Vector3d t_g_tr = t_global.translation();
		
		cout << t_g_tr.x() << " " << t_g_tr.y() << " " << t_g_tr.z() << " " << t_g_rot.x() << " " << t_g_rot.y() << " " << t_g_rot.z() << " " << t_g_rot.w() << "\n";
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
	if (argc < 2)
	{
		cerr << "Usage: " << argv[0] << " cloud_count [transform from gt to icp: t_x t_y t_z q_x q_y q_z q_w]\n";
		return 1;
	}
	maxCloudCount = atoi(argv[1]);
	if (argc >= 9)
	{
		trans_corr(0) = atof(argv[2]);
		trans_corr(1) = atof(argv[3]);
		trans_corr(2) = atof(argv[4]);
		rot_corr.x() = atof(argv[5]);
		rot_corr.y() = atof(argv[6]);
		rot_corr.z() = atof(argv[7]);
		rot_corr.w() = atof(argv[8]);
	}
	
	ros::init(argc, argv, "kinect_exp_logger");
	ros::NodeHandle n;
	tf::TransformListener listener;
	listenerPtr = &listener;
	
	ros::Subscriber sub = n.subscribe("/camera/depth/points2", 10, pointCloudCallback);

	ros::spin();
	
	dumpData();
	
	return 0;
}
