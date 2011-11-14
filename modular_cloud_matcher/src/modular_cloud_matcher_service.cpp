#include <fstream>

#include "ros/ros.h"
#include "pointmatcher/PointMatcher.h"

#include "aliases.h"
#include "get_params_from_server.h"
#include "cloud_conversion.h"
#include "ros_logger.h"

#include "modular_cloud_matcher/MatchClouds.h"

using namespace std;

class CloudMatcher
{
	ros::NodeHandle& n;
	
	PM::ICP icp;
	
	ros::ServiceServer service;
	
public:
	CloudMatcher(ros::NodeHandle& n);
	bool match(modular_cloud_matcher::MatchClouds::Request& req, modular_cloud_matcher::MatchClouds::Response& res);
};

CloudMatcher::CloudMatcher(ros::NodeHandle& n):
	n(n),
	service(n.advertiseService(getParam<string>("serviceName","matchClouds"), &CloudMatcher::match, this))
{
	// load config
	string configFileName;
	if (ros::param::get("~config", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icp.loadFromYaml(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load config from YAML file " << configFileName);
			icp.setDefault();
		}
	}
	else
	{
		ROS_WARN_STREAM("No config file specified, using default ICP chain.");
		icp.setDefault();
	}
	
	// replace logger
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);
}

bool CloudMatcher::match(modular_cloud_matcher::MatchClouds::Request& req, modular_cloud_matcher::MatchClouds::Response& res)
{
	// get and check reference
	size_t referenceGoodCount;
	DP referenceCloud(rosMsgToPointMatcherCloud(req.reference, referenceGoodCount));
	const unsigned referencePointCount(req.reference.width * req.reference.height);
	const double referenceGoodRatio(double(referenceGoodCount) / double(referencePointCount));
	
	if (referenceGoodCount == 0)
	{
		ROS_ERROR("I found no good points in the reference cloud");
		return false;
	}
	if (referenceGoodRatio < 0.5)
	{
		ROS_WARN_STREAM("Partial reference cloud! Missing " << 100 - referenceGoodRatio*100.0 << "% of the cloud (received " << referenceGoodCount << ")");
	}
	
	// get and check reading
	size_t readingGoodCount;
	DP readingCloud(rosMsgToPointMatcherCloud(req.readings, readingGoodCount));
	const unsigned readingPointCount(req.readings.width * req.readings.height);
	const double readingGoodRatio(double(readingGoodCount) / double(readingPointCount));
	
	if (readingGoodCount == 0)
	{
		ROS_ERROR("I found no good points in the reading cloud");
		return false;
	}
	if (readingGoodRatio < 0.5)
	{
		ROS_WARN_STREAM("Partial reference cloud! Missing " << 100 - readingGoodRatio*100.0 << "% of the cloud (received " << readingGoodCount << ")");
	}
	
	// call icp
	TP transform;
	try 
	{
		transform = icp(readingCloud, referenceCloud);
		ROS_INFO_STREAM("match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl);
	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return false;
	}
	
	// fill return value
	res.transform.translation.x = transform.coeff(0,3);
	res.transform.translation.y = transform.coeff(1,3);
	res.transform.translation.z = transform.coeff(2,3);
	const Eigen::Quaternion<Scalar> quat(Matrix3(transform.block(0,0,3,3)));
	res.transform.rotation.x = quat.x();
	res.transform.rotation.y = quat.y();
	res.transform.rotation.z = quat.z();
	res.transform.rotation.w = quat.w();
	
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_matcher_service");
	ros::NodeHandle n;
	
	CloudMatcher matcher(n);
	
	ros::spin();
	
	return 0;
}

