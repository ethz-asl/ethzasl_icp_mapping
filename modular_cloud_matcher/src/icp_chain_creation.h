#ifndef __ICP_CHAIN_CREATION_H
#define __ICP_CHAIN_CREATION_H

#include <boost/format.hpp>
#include "pointmatcher/PointMatcher.h"
#include "ros/ros.h"

// TODO: move this somewhere out
typedef float Scalar;
typedef MetricSpaceAligner<Scalar> MSA;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;


template<typename I, typename C>
I* createInstance(const std::string& root)
{
	return new C();
}

template<typename Interface>
struct Registrar
{
	//! A function which creates an instance of Interface
	typedef Interface* (*CreatorFunc)(const std::string& root);
	
	//! Register a class by storing a pointer to its constructor
	void reg(const std::string &name, const CreatorFunc func)
	{
		creators[name] = func;
	}
	
	//! Create an instance
	Interface* create(const std::string& name, const std::string& root)
	{
		if (creators.find(name) == creators.end())
		{
			std::cerr << "No element named " << name << " is registered. Known ones are:\n";
			dump(std::cerr);
			throw std::runtime_error("Trying to instanciate unknown element from registrar");
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
	typedef std::map<std::string, CreatorFunc> CreatorMap;
	CreatorMap creators;
};

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
	T v;
	if (ros::param::get(std::string("~")+name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}

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

MSA::DataPointsFilter* createClampOnAxisThresholdDataPointsFilter(const std::string& root)
{
	return new MSA::ClampOnAxisThresholdDataPointsFilter(
		getParam(root + "dim", 0),
		getParam(root + "threshold", 1.)
	);
}

MSA::DataPointsFilter* createClampOnAxisRatioDataPointsFilter(const std::string& root)
{
	return new MSA::ClampOnAxisRatioDataPointsFilter(
		getParam(root + "dim", 0),
		getParam(root + "ratio", 1.)
	);
}

MSA::DataPointsFilter* createSurfaceNormalDataPointsFilter(const std::string& root)
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

MSA::DataPointsFilter* createSamplingSurfaceNormalDataPointsFilter(const std::string& root)
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

MSA::DataPointsFilter* createRandomSamplingDataPointsFilter(const std::string& root)
{
	return new MSA::RandomSamplingDataPointsFilter(
		getParam(root + "prob", 0.5)
	);
}

MSA::DataPointsFilter* createFixstepSamplingDataPointsFilter(const std::string& root)
{
	return new MSA::FixstepSamplingDataPointsFilter(
		getParam(root + "startStep", 7.),
		getParam(root + "endStep", 7.),
		getParam(root + "stepMult", 1.)
	);
}

MSA::Matcher* createKDTreeMatcher(const std::string& root)
{
	return new MSA::KDTreeMatcher(
		getParam(root + "knn", 1),
		getParam(root + "epsilon", 0.)
	);
}

MSA::FeatureOutlierFilter* createMaxDistOutlierFilter(const std::string& root)
{
	return new MSA::MaxDistOutlierFilter(
		getParam(root + "maxDist", 1.)
	);
}

MSA::FeatureOutlierFilter* createMedianDistOutlierFilter(const std::string& root)
{
	return new MSA::MedianDistOutlierFilter(
		getParam(root + "factor", 3.)
	);
}

MSA::FeatureOutlierFilter* createTrimmedDistOutlierFilter(const std::string& root)
{
	return new MSA::TrimmedDistOutlierFilter(
		getParam(root + "factor", 0.9)
	);
}

MSA::TransformationChecker* createCounterTransformationChecker(const std::string& root)
{
	return new MSA::CounterTransformationChecker(
		getParam(root + "maxIterationCount", 60)
	);
}

MSA::TransformationChecker* createErrorTransformationChecker(const std::string& root)
{
	return new MSA::ErrorTransformationChecker(
		getParam(root + "minRotErrorDelta", 0.01),
		getParam(root + "minTransErrorDelta", 0.01),
		getParam(root + "tail", 3)
	);
}

MSA::TransformationChecker* createBoundTransformationChecker(const std::string& root)
{
	return new MSA::BoundTransformationChecker(
		getParam(root + "maxRotationNorm", 1.),
		getParam(root + "maxTranslationNorm", 1.)
	);
}

MSA::Inspector* createVTKFileInspector(const std::string& root)
{
	return new MSA::VTKFileInspector(
		getParam(root + "baseFileName", std::string("msa-output"))
	);
}

typedef MSA::TransformationParameters TP;
typedef MSA::DataPoints DP;

void initParameters()
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
}

void populateParameters(MSA::ICPSequence& icp)
{
	// global parameters
	icp.ratioToSwitchKeyframe = getParam("ratioToSwitchKeyframe", 0.8);
	
	// icp parameters
	const int transformationCount(getParam<int>("transformationCount", 1));
	const int readingDataPointsFilterCount(getParam<int>("readingDataPointsFilterCount", 1));
	const int readingStepDataPointsFilterCount(getParam<int>("readingStepDataPointsFilterCount", 0));
	const int keyframeDataPointsFilterCount(getParam<int>("keyframeDataPointsFilterCount", 1));
	const int featureOutlierFilterCount(getParam<int>("featureOutlierFilterCount", 1));
	const int transformationCheckerCount(getParam<int>("transformationCheckerCount", 1));
	
	for (int i = 0; i < readingDataPointsFilterCount; ++i)
	{
		std::string root((boost::format("readingDataPointsFilter/%1%") % i).str());
		icp.readingDataPointsFilters.push_back(REG(DataPointsFilter).create(getParam<std::string>(root+"/name", "FixstepSamplingDataPointsFilter"), root + "/"));
	}
	for (int i = 0; i < readingStepDataPointsFilterCount; ++i)
	{
		std::string root((boost::format("readingStepDataPointsFilter/%1%") % i).str());
		icp.readingStepDataPointsFilters.push_back(REG(DataPointsFilter).create(getParam<std::string>(root+"/name", "FixstepSamplingDataPointsFilter"), root + "/"));
	}
	for (int i = 0; i < keyframeDataPointsFilterCount; ++i)
	{
		std::string root((boost::format("keyframeDataPointsFilter/%1%") % i).str());
		icp.keyframeDataPointsFilters.push_back(REG(DataPointsFilter).create(getParam<std::string>(root+"/name", "SamplingSurfaceNormalDataPointsFilter"), root + "/"));
	}
	for (int i = 0; i < transformationCount; ++i)
	{
		std::string root((boost::format("transformations/%1%") % i).str());
		icp.transformations.push_back(REG(Transformation).create(getParam<std::string>(root+"/name", "TransformFeatures"), root + "/"));
	}
	icp.matcher = REG(Matcher).create(getParam<std::string>("matcher/name", "KDTreeMatcher"), "matcher/");
	for (int i = 0; i < featureOutlierFilterCount; ++i)
	{
		std::string root((boost::format("featureOutlierFilters/%1%") % i).str());
		icp.featureOutlierFilters.push_back(REG(FeatureOutlierFilter).create(getParam<std::string>(root+"/name", "MedianDistOutlierFilter"), root + "/"));
	}
	icp.descriptorOutlierFilter = REG(DescriptorOutlierFilter).create(getParam<std::string>("descriptorOutlierFilter/name", "NullDescriptorOutlierFilter"), "descriptorOutlierFilter/");
	icp.errorMinimizer = REG(ErrorMinimizer).create(getParam<std::string>("errorMinimizer/name", "PointToPlaneErrorMinimizer"), "errorMinimizer/");
	for (int i = 0; i < transformationCheckerCount; ++i)
	{
		std::string root((boost::format("transformationCheckers/%1%") % i).str());
		icp.transformationCheckers.push_back(REG(TransformationChecker).create(getParam<std::string>(root+"/name", "ErrorTransformationChecker"), root + "/"));
	}
	icp.inspector = REG(Inspector).create(getParam<std::string>("inspector/name", "Inspector"), "inspector");
	//icp.inspector = REG(Inspector).create(getParam<std::string>("inspector", "VTKFileInspector"), "inspector");
	icp.outlierMixingWeight = getParam<double>("outlierMixingWeight", 1);
}

#endif // __ICP_CHAIN_CREATION_H