#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"

#include "aliases.h"
#include "get_params_from_server.h"
#include "cloud_conversion.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"

using namespace std;

namespace PointMatcherSupport
{
	struct ROSLogger: public Logger
	{
		virtual bool hasInfoChannel() const{ return true; };
		virtual void beginInfoEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* infoStream() { return &_infoStream; }
		virtual void finishInfoEntry(const char *file, unsigned line, const char *func);
		virtual bool hasWarningChannel() const { return true; }
		virtual void beginWarningEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* warningStream() { return &_warningStream; }
		virtual void finishWarningEntry(const char *file, unsigned line, const char *func);
		
	protected:
		void writeRosLog(ros::console::Level level, const char* file, int line, const char *func, const string& text);
		
		std::ostringstream _infoStream;
		std::ostringstream _warningStream;
	};
	
	void ROSLogger::beginInfoEntry(const char *file, unsigned line, const char *func)
	{
		_infoStream.str("");
	}
	
	void ROSLogger::finishInfoEntry(const char *file, unsigned line, const char *func)
	{
		writeRosLog(ros::console::levels::Info, file, line, func, _infoStream.str());
	}
	
	void ROSLogger::beginWarningEntry(const char *file, unsigned line, const char *func)
	{
		_warningStream.str("");
	}
	
	void ROSLogger::finishWarningEntry(const char *file, unsigned line, const char *func)
	{
		writeRosLog(ros::console::levels::Warn, file, line, func, _warningStream.str());
	}
	
	void ROSLogger::writeRosLog(ros::console::Level level, const char* file, int line, const char *func, const string& text)
	{
		ROSCONSOLE_DEFINE_LOCATION(true, level, ROSCONSOLE_DEFAULT_NAME);
		if (enabled)
			ros::console::print(0, loc.logger_, loc.level_, file, line, func, "%s", text.c_str());
	}
};

class CloudMatcher
{
	ros::NodeHandle& n;
	
	PM::ICPSequence icp;
	
	const string fixedFrame;
	const string sensorFrame;
	const unsigned startupDropCount;
	unsigned dropCount;
	
	ros::Subscriber cloudSub;
	ros::Publisher pathPub;
	nav_msgs::Path path;
	tf::TransformBroadcaster br;
	ros::Publisher posePub;
	
public:
	CloudMatcher(ros::NodeHandle& n);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsg);
};

CloudMatcher::CloudMatcher(ros::NodeHandle& n):
	n(n),
	fixedFrame(getParam<string>("fixedFrame",  "/world")),
	sensorFrame(getParam<string>("sensorFrame",  "/openni_rgb_optical_frame")),
	startupDropCount(getParam("startupDropCount", 0)),
	dropCount(0)
{
	const string cloudTopic(getParam<string>("cloudTopic", "/camera/rgb/points"));
	cloudSub = n.subscribe(cloudTopic, 1, &CloudMatcher::gotCloud, this);
	
	const string pathTopic(getParam<string>("path", "/tracker_path"));
	pathPub = n.advertise<nav_msgs::Path>(pathTopic, 1);
	
	path.header.frame_id = fixedFrame;

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
	//PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);
	
	posePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(getParam<string>("deltaPoseTopic", "/openni_delta_pose"), 3);
}

void CloudMatcher::gotCloud(const sensor_msgs::PointCloud2& cloudMsg)
{
	if (dropCount < startupDropCount)
	{
		++dropCount;
		return;
	}
	
	size_t goodCount(0);
	DP dp(rosMsgToPointMatcherCloud(cloudMsg, goodCount));
	
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		if (posePub.getNumSubscribers() > 0)
		{
			geometry_msgs::PoseWithCovarianceStamped pose;
			geometry_msgs::Point& position(pose.pose.pose.position);
			geometry_msgs::Quaternion& orientation(pose.pose.pose.orientation);
			
			pose.header.stamp = cloudMsg.header.stamp;
			
			position.x = numeric_limits<float>::quiet_NaN();
			position.y = numeric_limits<float>::quiet_NaN();
			position.z = numeric_limits<float>::quiet_NaN();
			orientation.x = numeric_limits<float>::quiet_NaN();
			orientation.y = numeric_limits<float>::quiet_NaN();
			orientation.z = numeric_limits<float>::quiet_NaN();
			orientation.w = numeric_limits<float>::quiet_NaN();
			
			posePub.publish(pose);
		}
		return;
	}
	
	const unsigned pointCount(cloudMsg.width * cloudMsg.height);
	//ROS_INFO_STREAM("Got " << pointCount << " points (" << goodCount << " goods)");
	const double imageRatio = (double)goodCount / (double)pointCount;
	
	//TODO: put that as parameter, tricky to set...
	if (imageRatio < 0.5)
	{
		ROS_WARN_STREAM("Partial cloud! Missing " << 100 - imageRatio*100.0 << "% of the cloud (received " << goodCount << ")");
		//return;
	}
	
	// call icp
	bool icpWasSuccess(true);
	try 
	{
		icp(dp);
		//ROS_INFO_STREAM("match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl);
	}
	catch (PM::ConvergenceError error)
	{
		icpWasSuccess = false;
		ROS_WARN_STREAM("ICP failed to converge: " << error.what());
	}
	
	// broadcast transform
	if (posePub.getNumSubscribers() > 0)
	{
		const TP dTransform(icp.getDeltaTransform());
		const Eigen::eigen2_Quaternion<Scalar> dTquat(Matrix3(dTransform.block(0,0,3,3)));
		const Vector3 dTtr(dTransform.block(0,3,3,1));
		
		geometry_msgs::PoseWithCovarianceStamped pose;
		geometry_msgs::Point& position(pose.pose.pose.position);
		geometry_msgs::Quaternion& orientation(pose.pose.pose.orientation);
		
		pose.header.stamp = cloudMsg.header.stamp;
		
		if (icpWasSuccess)
		{
			position.x = dTtr(0);
			position.y = dTtr(1);
			position.z = dTtr(2);
			orientation.x = dTquat.x();
			orientation.y = dTquat.y();
			orientation.z = dTquat.z();
			orientation.w = dTquat.w();
		}
		else
		{
			ROS_WARN_STREAM("ICP failure in sendDeltaPose mode, resetting tracker");
			// we have a failure, so we are sure that we did not create a key frame, 
			// so dp is not affected. We can thus create a new keyframe
			icp.resetTracking(dp);
			position.x = numeric_limits<float>::quiet_NaN();
			position.y = numeric_limits<float>::quiet_NaN();
			position.z = numeric_limits<float>::quiet_NaN();
			orientation.x = numeric_limits<float>::quiet_NaN();
			orientation.y = numeric_limits<float>::quiet_NaN();
			orientation.z = numeric_limits<float>::quiet_NaN();
			orientation.w = numeric_limits<float>::quiet_NaN();
		}
		posePub.publish(pose);
	}
	
	// FIXME: should we continue publishing absolute pose as tf in sendDeltaPoseMessage mode?
	
	const TP globalTransform(icp.getTransform());
	tf::Quaternion tfQuat;
#if ROS_VERSION_MINIMUM(1, 6, 0)
	// electric and later
	const Eigen::Quaternion<Scalar> quat(Matrix3(globalTransform.block(0,0,3,3)));
	tf::RotationEigenToTF(quat.cast<double>(), tfQuat);
#else // ROS_VERSION_MINIMUM(1, 6, 0)
	// diamondback and before
	const Eigen::eigen2_Quaternion<Scalar> quat(Matrix3(globalTransform.block(0,0,3,3)));
	tf::RotationEigenToTF(quat.cast<double>(), tfQuat);
#endif // ROS_VERSION_MINIMUM(1, 6, 0)
	tf::Vector3 tfVect;
	tf::VectorEigenToTF(globalTransform.block(0,3,3,1).cast<double>(), tfVect);
	tf::Transform transform;
	transform.setRotation(tfQuat);
	transform.setOrigin(tfVect);

	if (icp.keyFrameCreatedAtLastCall())
	{
		ROS_INFO_STREAM("Keyframe created at " << icp.errorMinimizer->getWeightedPointUsedRatio());
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
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_matcher_node");
	ros::NodeHandle n;
	
	CloudMatcher matcher(n);
	
	ros::spin();
	
	return 0;
}
