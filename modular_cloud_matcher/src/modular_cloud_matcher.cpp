#include <boost/format.hpp>

#include "ros/ros.h"
#include "modular_cloud_matcher/MatchClouds.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "pointmatcher/PointMatcher.h"

using namespace std;

template<typename I, typename C>
I* createInstance(const std::string& root)
{
	return new C();
}

template<typename Interface>
struct Registrar
{
	//! A function which creates an instance of Interface
	typedef Interface* (*CreatorFunc)(const string& root);
	
	//! Register a class by storing a pointer to its constructor
	void reg(const string &name, const CreatorFunc func)
	{
		creators[name] = func;
	}
	
	//! Create an instance
	Interface* create(const string& name, const string& root)
	{
		if (creators.find(name) == creators.end())
		{
			cerr << "No element named " << name << " is registered. Known ones are:\n";
			dump(cerr);
			throw runtime_error("Trying to instanciate unknown element from registrar");
		}
		//cerr << "Creating object " << name << endl;
		return creators[name](root);
	}
	
	//! Print the list of registered classes to stream
	void dump(std::ostream &stream)
	{
		for (typename CreatorMap::const_iterator it = creators.begin(); it != creators.end(); ++it)
			stream << "- " << it->first << "\n";
	}
	
protected:
	typedef map<string, CreatorFunc> CreatorMap;
	CreatorMap creators;
};

template<typename T>
T getParam(const string& name, const T& defaultValue)
{
	T v;
	if (ros::param::get(string("~")+name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}

typedef float Scalar;
typedef MetricSpaceAligner<Scalar> MSA;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;


#define REG(name) (name##Registrar)
#define DEF_REGISTRAR(name) Registrar< MSA::name > name##Registrar;
#define ADD_TO_REGISTRAR(name, element) name##Registrar.reg(# element, &createInstance<MSA::name, MSA::element>);
#define ADD_CUSTOM_TO_REGISTRAR(name, element, cFunc) name##Registrar.reg(# element, &cFunc);

DEF_REGISTRAR(Transformation)
DEF_REGISTRAR(DataPointsFilter)
DEF_REGISTRAR(Matcher)
DEF_REGISTRAR(FeatureOutlierFilter)
DEF_REGISTRAR(DescriptorOutlierFilter)
DEF_REGISTRAR(ErrorMinimizer)
DEF_REGISTRAR(TransformationChecker)
DEF_REGISTRAR(Inspector)

MSA::DataPointsFilter* createClampOnAxisThresholdDataPointsFilter(const string& root)
{
	return new MSA::ClampOnAxisThresholdDataPointsFilter(
		getParam(root + "dim", 0),
		getParam(root + "threshold", 1.)
	);
}

MSA::DataPointsFilter* createClampOnAxisRatioDataPointsFilter(const string& root)
{
	return new MSA::ClampOnAxisRatioDataPointsFilter(
		getParam(root + "dim", 0),
		getParam(root + "ratio", 1.)
	);
}

MSA::DataPointsFilter* createSurfaceNormalDataPointsFilter(const string& root)
{
	return new MSA::SurfaceNormalDataPointsFilter(
		getParam(root + "knn", 10),
		getParam(root + "epsilon", 0.),
		getParam(root + "keepNormals", true),
		getParam(root + "keepDensities", false),
		getParam(root + "keepEigenValues", false),
		getParam(root + "keepEigenVectors", false)
	);
}

MSA::DataPointsFilter* createSamplingSurfaceNormalDataPointsFilter(const string& root)
{
	return new MSA::SamplingSurfaceNormalDataPointsFilter(
		getParam(root + "k", 20),
		getParam(root + "averageExistingDescriptors", true),
		getParam(root + "keepNormals", true),
		getParam(root + "keepDensities", false),
		getParam(root + "keepEigenValues", false),
		getParam(root + "keepEigenVectors", false)
	);
}

MSA::DataPointsFilter* createRandomSamplingDataPointsFilter(const string& root)
{
	return new MSA::RandomSamplingDataPointsFilter(
		getParam(root + "prob", 0.5)
	);
}

MSA::DataPointsFilter* createFixstepSamplingDataPointsFilter(const string& root)
{
	return new MSA::FixstepSamplingDataPointsFilter(
		getParam(root + "step", 20)
	);
}

MSA::Matcher* createKDTreeMatcher(const string& root)
{
	return new MSA::KDTreeMatcher(
		getParam(root + "knn", 1),
		getParam(root + "epsilon", 0.)
	);
}

MSA::FeatureOutlierFilter* createMaxDistOutlierFilter(const string& root)
{
	return new MSA::MaxDistOutlierFilter(
		getParam(root + "maxDist", 1.)
	);
}

MSA::FeatureOutlierFilter* createMedianDistOutlierFilter(const string& root)
{
	return new MSA::MedianDistOutlierFilter(
		getParam(root + "factor", 3.)
	);
}

MSA::FeatureOutlierFilter* createTrimmedDistOutlierFilter(const string& root)
{
	return new MSA::TrimmedDistOutlierFilter(
		getParam(root + "factor", 0.9)
	);
}

MSA::TransformationChecker* createCounterTransformationChecker(const string& root)
{
	return new MSA::CounterTransformationChecker(
		getParam(root + "maxIterationCount", 60)
	);
}

MSA::TransformationChecker* createErrorTransformationChecker(const string& root)
{
	return new MSA::ErrorTransformationChecker(
		getParam(root + "minRotErrorDelta", 0.01),
		getParam(root + "minTransErrorDelta", 0.01),
		getParam(root + "tail", 3)
	);
}

MSA::TransformationChecker* createBoundTransformationChecker(const string& root)
{
	return new MSA::BoundTransformationChecker(
		getParam(root + "maxRotationNorm", 1.),
		getParam(root + "maxTranslationNorm", 1.)
	);
}

MSA::Inspector* createVTKFileInspector(const string& root)
{
	return new MSA::VTKFileInspector(
		getParam(root + "baseFileName", string("msa-output"))
	);
}

typedef MSA::TransformationParameters TP;
typedef MSA::DataPoints DP;

MSA::ICPSequence icp(3);

void populateParameters(void)
{
	// global parameters
	icp.ratioToSwitchKeyframe = getParam("ratioToSwitchKeyframe", 0.8);
	
	// icp parameters
	const int transformationCount(getParam<int>("transformationCount", 1));
	const int readingDataPointsFilterCount(getParam<int>("readingDataPointsFilterCount", 1));
	const int keyframeDataPointsFilterCount(getParam<int>("keyframeDataPointsFilterCount", 1));
	const int featureOutlierFilterCount(getParam<int>("featureOutlierFilterCount", 1));
	const int transformationCheckerCount(getParam<int>("transformationCheckerCount", 1));
	
	for (int i = 0; i < readingDataPointsFilterCount; ++i)
	{
		string root((boost::format("readingDataPointsFilter/%1%") % i).str());
		icp.readingDataPointsFilters.push_back(REG(DataPointsFilter).create(getParam<string>(root+"/name", "FixstepSamplingDataPointsFilter"), root + "/"));
	}
	for (int i = 0; i < keyframeDataPointsFilterCount; ++i)
	{
		string root((boost::format("keyframeDataPointsFilter/%1%") % i).str());
		icp.keyframeDataPointsFilters.push_back(REG(DataPointsFilter).create(getParam<string>(root+"/name", "SamplingSurfaceNormalDataPointsFilter"), root + "/"));
	}
	for (int i = 0; i < transformationCount; ++i)
	{
		string root((boost::format("transformations/%1%") % i).str());
		icp.transformations.push_back(REG(Transformation).create(getParam<string>(root+"/name", "TransformFeatures"), root + "/"));
	}
	icp.matcher = REG(Matcher).create(getParam<string>("matcher/name", "KDTreeMatcher"), "matcher/");
	for (int i = 0; i < featureOutlierFilterCount; ++i)
	{
		string root((boost::format("featureOutlierFilters/%1%") % i).str());
		icp.featureOutlierFilters.push_back(REG(FeatureOutlierFilter).create(getParam<string>(root+"/name", "MedianDistOutlierFilter"), root + "/"));
	}
	icp.descriptorOutlierFilter = REG(DescriptorOutlierFilter).create(getParam<string>("descriptorOutlierFilter/name", "NullDescriptorOutlierFilter"), "descriptorOutlierFilter/");
	icp.errorMinimizer = REG(ErrorMinimizer).create(getParam<string>("errorMinimizer/name", "PointToPlaneErrorMinimizer"), "errorMinimizer/");
	for (int i = 0; i < transformationCheckerCount; ++i)
	{
		string root((boost::format("transformationCheckers/%1%") % i).str());
		icp.transformationCheckers.push_back(REG(TransformationChecker).create(getParam<string>(root+"/name", "ErrorTransformationChecker"), root + "/"));
	}
	icp.inspector = REG(Inspector).create(getParam<string>("inspector/name", "Inspector"), "inspector");
	//icp.inspector = REG(Inspector).create(getParam<string>("inspector", "VTKFileInspector"), "inspector");
	icp.outlierMixingWeight = getParam<double>("outlierMixingWeight", 1);
}

static DP keyFrameCloud;
static TP keyFrameTransform(TP::Identity(4, 4));
static TP curTransform(TP::Identity(4, 4));

static Vector3 sumRot(Vector3::Zero(3));
static Vector3 sumTrans(Vector3::Zero(3));

static Scalar maxSensorDist(-1);
static string fixedFrame;
static string sensorFrame;

nav_msgs::Path path;
ros::Publisher pathPub;

void gotCloud(const sensor_msgs::PointCloud& cloudMsg)
{
	// create labels
	DP::Labels labels;
	labels.push_back(DP::Label("x", 1));
	labels.push_back(DP::Label("y", 1));
	labels.push_back(DP::Label("z", 1));
	labels.push_back(DP::Label("pad", 1));
	
	// create data points
	size_t goodCount(0);
	for (size_t i = 0; i < cloudMsg.points.size(); ++i)
	{
		// FIXME: kinect-specific hack, do something better
		if (!isnan(cloudMsg.points[i].x))
			++goodCount;
	}
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	DP dp(DP::Features(4, goodCount), labels);
	int dIndex(0);
	for (size_t i = 0; i < cloudMsg.points.size(); ++i)
	{
		// FIXME: kinect-specific hack, do something better
		if (!isnan(cloudMsg.points[i].x))
		{
			dp.features.coeffRef(0, dIndex) = cloudMsg.points[i].x;
			dp.features.coeffRef(1, dIndex) = cloudMsg.points[i].y;
			dp.features.coeffRef(2, dIndex) = cloudMsg.points[i].z;
			dp.features.coeffRef(3, dIndex) = 1;
			++dIndex;
		}
	}
	ROS_INFO_STREAM("Got " << cloudMsg.points.size() << " points (" << goodCount << " goods)");
	
	const double imageRatio = (double)goodCount / (double)cloudMsg.points.size();
	
	//TODO: put that as parameter, tricky to set...
	if (goodCount < 8000)
	{
		ROS_ERROR_STREAM("Partial image! Missing " << 100 - imageRatio*100.0 << "% of the image (received " << goodCount << ")");
		return;
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
	const Eigen::Quaternion<Scalar> quat(Matrix3(globalTransform.block(0,0,3,3)));
	//cerr << "pos:\n" << curTransform << endl;
	const Scalar a0(atan2(2*(quat.w() * quat.x() + quat.y() * quat.z()), 1-2*(quat.x() * quat.x() + quat.y() * quat.y())));
	const Scalar a1(asin(2*(quat.w() * quat.y() - quat.z() * quat.x())));
	const Scalar a2(atan2(2*(quat.w() * quat.z() + quat.x() * quat.y()), 1-2*(quat.y() * quat.y() + quat.z() * quat.z())));
	
	sumRot += Vector3(a0, a1, a2);
	sumTrans += curTransform.block(0,3,3,1);
	
	//cerr << "rot mean: " << (sumRot/double(cnt-1)).transpose() << endl;
	//cerr << "trans mean: " << (sumTrans/double(cnt-1)).transpose() << endl;
	
	//cerr << "rot: " << a0 << " " << a1 << " " << a2 << endl;
	
	tf::Quaternion tfQuat;
	tf::RotationEigenToTF(Eigen::Quaterniond(quat), tfQuat);
	tf::Vector3 tfVect;
	tf::VectorEigenToTF(globalTransform.block(0,3,3,1).cast<double>(), tfVect);
	tf::Transform transform;
	transform.setRotation(tfQuat);
	transform.setOrigin(tfVect);
	//transform.setOrigin(tf::Vector3(0,0,0));
	//transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
	
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
	
	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(transform, cloudMsg.header.stamp, fixedFrame, sensorFrame));
}

int main(int argc, char **argv)
{
	ADD_TO_REGISTRAR(Transformation, TransformFeatures)
	ADD_TO_REGISTRAR(Transformation, TransformDescriptors)
	
	ADD_TO_REGISTRAR(DataPointsFilter, IdentityDataPointsFilter)
	ADD_CUSTOM_TO_REGISTRAR(DataPointsFilter, ClampOnAxisThresholdDataPointsFilter, createClampOnAxisThresholdDataPointsFilter)
	ADD_CUSTOM_TO_REGISTRAR(DataPointsFilter, ClampOnAxisRatioDataPointsFilter, createClampOnAxisRatioDataPointsFilter)
	ADD_CUSTOM_TO_REGISTRAR(DataPointsFilter, SurfaceNormalDataPointsFilter, createSurfaceNormalDataPointsFilter)
	ADD_CUSTOM_TO_REGISTRAR(DataPointsFilter, SamplingSurfaceNormalDataPointsFilter, createSamplingSurfaceNormalDataPointsFilter)
	ADD_CUSTOM_TO_REGISTRAR(DataPointsFilter, RandomSamplingDataPointsFilter, createRandomSamplingDataPointsFilter)
	ADD_CUSTOM_TO_REGISTRAR(DataPointsFilter, FixstepSamplingDataPointsFilter, createFixstepSamplingDataPointsFilter)
	
	ADD_TO_REGISTRAR(Matcher, NullMatcher)
	ADD_CUSTOM_TO_REGISTRAR(Matcher, KDTreeMatcher, createKDTreeMatcher)
	ADD_TO_REGISTRAR(FeatureOutlierFilter, NullFeatureOutlierFilter)
	
	ADD_CUSTOM_TO_REGISTRAR(FeatureOutlierFilter, MaxDistOutlierFilter, createMaxDistOutlierFilter)
	ADD_CUSTOM_TO_REGISTRAR(FeatureOutlierFilter, MedianDistOutlierFilter, createMedianDistOutlierFilter)
	ADD_CUSTOM_TO_REGISTRAR(FeatureOutlierFilter, TrimmedDistOutlierFilter, createTrimmedDistOutlierFilter)
	
	ADD_TO_REGISTRAR(DescriptorOutlierFilter, NullDescriptorOutlierFilter)
	
	ADD_TO_REGISTRAR(ErrorMinimizer, IdentityErrorMinimizer)
	ADD_TO_REGISTRAR(ErrorMinimizer, PointToPointErrorMinimizer)
	ADD_TO_REGISTRAR(ErrorMinimizer, PointToPlaneErrorMinimizer)
	
	ADD_CUSTOM_TO_REGISTRAR(TransformationChecker, CounterTransformationChecker, createCounterTransformationChecker)
	ADD_CUSTOM_TO_REGISTRAR(TransformationChecker, ErrorTransformationChecker, createErrorTransformationChecker)
	ADD_CUSTOM_TO_REGISTRAR(TransformationChecker, BoundTransformationChecker, createBoundTransformationChecker)
	
	ADD_TO_REGISTRAR(Inspector, Inspector)
	ADD_CUSTOM_TO_REGISTRAR(Inspector, VTKFileInspector, createVTKFileInspector)
	
	ros::init(argc, argv, "cloud_matcher");
	ros::NodeHandle n;
	
	string cloudTopic(getParam<string>("cloudTopic", "/camera/depth/points"));
	ros::Subscriber sub = n.subscribe(cloudTopic, 1, gotCloud);
	string pathTopic(getParam<string>("path", "/tracker_path"));
	pathPub = n.advertise<nav_msgs::Path>(pathTopic, 1);
	
	maxSensorDist = getParam("maxSensorDist", 2.5);
	fixedFrame = getParam<string>("fixedFrame",  "/world");
	sensorFrame = getParam<string>("sensorFrame",  "/openni_rgb_optical_frame");
	populateParameters();
	
	path.header.frame_id = fixedFrame;
	
	ros::spin();
	
	return 0;
}


#if 0
bool matchClouds(modular_cloud_matcher::MatchClouds::Request& req,
				 modular_cloud_matcher::MatchClouds::Response& res)
{
	//ros::param::param("featureTransformation", default_param, "default_value");
	
	// create labels
	DP::Labels labels;
	labels.push_back(DP::Label("x", 1));
	labels.push_back(DP::Label("y", 1));
	labels.push_back(DP::Label("z", 1));
	labels.push_back(DP::Label("pad", 1));
	
	// create reference
	DP ref(DP::Features(4, req.reference.points.size()), labels);
	for (size_t i = 0; i < req.reference.points.size(); ++i)
	{
		ref.features.coeffRef(0, i) = req.reference.points[i].x;
		ref.features.coeffRef(1, i) = req.reference.points[i].y;
		ref.features.coeffRef(2, i) = req.reference.points[i].z;
		ref.features.coeffRef(3, i) = 1;
	}
	ROS_INFO_STREAM("reference has " << ref.features.cols() << " points");
	
	// create readings
	DP data(DP::Features(4, req.readings.points.size()), labels);
	for (size_t i = 0; i < req.readings.points.size(); ++i)
	{
		data.features.coeffRef(0, i) = req.readings.points[i].x;
		data.features.coeffRef(1, i) = req.readings.points[i].y;
		data.features.coeffRef(2, i) = req.readings.points[i].z;
		data.features.coeffRef(3, i) = 1;
	}
	ROS_INFO_STREAM("readings has " << data.features.cols() << " points");
	
	// do icp
	const TP initialTransform(TP::Identity(data.features.rows(), data.features.rows()));
	const TP finalTransform = MSA::icp(initialTransform, data, ref, p);
	const Eigen::Quaternion<double> quat(Eigen::Matrix3d(finalTransform.block(0,0,3,3)));
	
	res.transform.translation.x = finalTransform.coeff(0,3);
	res.transform.translation.y = finalTransform.coeff(1,3);
	res.transform.translation.z = finalTransform.coeff(2,3);
	res.transform.rotation.x = quat.x();
	res.transform.rotation.y = quat.y();
	res.transform.rotation.z = quat.z();
	res.transform.rotation.w = quat.w();
	
	return true;
}
#endif
