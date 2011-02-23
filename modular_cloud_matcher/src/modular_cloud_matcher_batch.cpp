#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

typedef map<string, string> Params;

Params params;

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
	Params::const_iterator it(params.find(name));
	if (it != params.end())
	{
		T v;
		istringstream iss(it->second);
		iss >> v;
		cerr << "Found parameter: " << name << ", value: " << v;
		return v;
	}
	else
	{
		cerr << "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue;
		return defaultValue;
	}
}

#include "icp_chain_creation.h"
#include "pointmatcher/PointMatcher.h"

struct Datum
{
	TP transform;
	DP cloud;
};

typedef vector<Datum> Data;

Data data;

int main(int argc, char **argv)
{
	if (argc != 5)
	{
		cerr << "Usage: " << argv[0] << " data params_names params_values output" << endl;
		return 1;
	}
	
	// load data
	{
		// create labels
		DP::Labels labels;
		labels.push_back(DP::Label("x", 1));
		labels.push_back(DP::Label("y", 1));
		labels.push_back(DP::Label("z", 1));
		labels.push_back(DP::Label("pad", 1));
	
		// open file
		ifstream ifs(argv[1]);
		if (!ifs.good())
		{
			cerr << "Error, invalid data file " << argv[1] << endl;
		}
		char line[1024];
		
		// file header
		ifs.getline(line, 1024);
		cout << "Header: " << line << endl;
		// cloud count
		size_t cloudCount;
		ifs >> cloudCount;
		ifs.getline(line, 1024);
		cout << "Reading " << cloudCount << " clouds\n\n";
		data.reserve(cloudCount);
		for (size_t i = 0; i < cloudCount; ++i)
		{
			Datum datum;
			// cloud header
			ifs.getline(line, 1024);
			cout << "Cloud header: " << line << endl;
			// transform
			Scalar t_x, t_y, t_z, q_x,q_y,q_z,q_w;
			ifs >> t_x;
			ifs >> t_y;
			ifs >> t_z;
			ifs >> q_x;
			ifs >> q_y;
			ifs >> q_z;
			ifs >> q_w;
			cout << "Transform " << i << ": trans=(" << t_x << "," << t_y << "," << t_z << "), quat=(" << q_x << "," << q_y << "," << q_z << "," << q_w << ")\n";
			const Vector3 vect(t_x, t_y, t_z);
			const Quaternion<Scalar> quat(q_w, q_x, q_y, q_z);
			const Transform<Scalar,3,Affine> tr = Translation<Scalar,3>(vect) * quat;
			datum.transform = tr.matrix();
			// point count
			size_t pointCount;
			ifs >> pointCount;
			DP::Features tempCloud(4, pointCount);
			int dIndex(0);
			for (size_t j = 0; j < pointCount; ++j)
			{
				string sx, sy, sz;
				ifs >> sx;
				ifs >> sy;
				ifs >> sz;
				if (sx != "nan" && sy != "nan" && sz != "nan")
				{
					tempCloud(0, dIndex) = atof(sx.c_str());
					tempCloud(1, dIndex) = atof(sy.c_str());
					tempCloud(2, dIndex) = atof(sz.c_str());
					//cout << t_x << " " << t_y << " " << t_z << "\n";
					tempCloud(3, dIndex) = 1;
					++dIndex;
				}
				else
				{
					//cout << "nan at pos " << j << "\n";
				}
			}
			cout << "Cloud " << i << ", " << dIndex << " good points on " << pointCount << "\n\n";
			datum.cloud = DP(tempCloud.leftCols(dIndex), labels);
			data.push_back(datum);
			ifs.getline(line, 1024);
		}
	}
	
	// load param list
	
	/*if (dropCount < startupDropCount)
	{
		++dropCount;
		return;
	}
	
	// create labels
	DP::Labels labels;
	labels.push_back(DP::Label("x", 1));
	labels.push_back(DP::Label("y", 1));
	labels.push_back(DP::Label("z", 1));
	labels.push_back(DP::Label("pad", 1));
	
	// create data points
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (cloudMsg, cloud);

	size_t goodCount(0);
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		if (!isnan(cloud.points[i].x))
			++goodCount;
	}
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	DP dp(DP::Features(4, goodCount), labels);
	int dIndex(0);
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		if (!isnan(cloud.points[i].x))
		{
			dp.features.coeffRef(0, dIndex) = cloud.points[i].x;
			dp.features.coeffRef(1, dIndex) = cloud.points[i].y;
			dp.features.coeffRef(2, dIndex) = cloud.points[i].z;
			dp.features.coeffRef(3, dIndex) = 1;
			++dIndex;
		}
	}
	ROS_INFO_STREAM("Got " << cloud.points.size() << " points (" << goodCount << " goods)");
	
	const double imageRatio = (double)goodCount / (double)cloud.points.size();
	
	//TODO: put that as parameter, tricky to set...
	if (goodCount < 10000)
	{
		ROS_ERROR_STREAM("Partial image! Missing " << 100 - imageRatio*100.0 << "% of the image (received " << goodCount << ")");
		//return;
	}
	
	// call icp
	try 
	{
		icp(dp);
		ROS_INFO_STREAM("match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl);
	}
	catch (MSA::ConvergenceError error)
	{
		ROS_WARN_STREAM("ICP failed to converge: " << error.what());
	}
	
	// broadcast transform
	const TP globalTransform(icp.getTransform());
	const Eigen::eigen2_Quaternion<Scalar> quat(Matrix3(globalTransform.block(0,0,3,3)));
	
	tf::Quaternion tfQuat;
	tf::RotationEigenToTF(quat.cast<double>(), tfQuat);
	tf::Vector3 tfVect;
	tf::VectorEigenToTF(globalTransform.block(0,3,3,1).cast<double>(), tfVect);
	tf::Transform transform;
	transform.setRotation(tfQuat);
	transform.setOrigin(tfVect);
	
	if (icp.keyFrameCreatedAtLastCall())
	{
		ROS_WARN_STREAM("Keyframe created at " << icp.errorMinimizer->getWeightedPointUsedRatio());
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = sensorFrame;
		pose.pose.position.x = tfVect.x();
		pose.pose.position.y = tfVect.y();
		pose.pose.position.z = tfVect.z();
		pose.pose.orientation.x = tfQuat.x();
		pose.pose.orientation.y = tfQuat.y();
		pose.pose.orientation.z = tfQuat.z();
		pose.pose.orientation.w = tfQuat.w();
		path.poses.push_back(pose);
		pathPub.publish(path);
	}
	
	br.sendTransform(tf::StampedTransform(transform, cloudMsg.header.stamp, fixedFrame, sensorFrame));
	*/

	return 0;
}
