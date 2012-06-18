#include "pointmatcher_ros/point_cloud.h"
#include "ros/ros.h"
#include "boost/detail/endian.hpp"
#include <vector>
#include <memory>

namespace PointMatcher_ros
{
	using namespace std;
	
	//! Transform a ROS PointCloud2 message into a libpointmatcher point cloud
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg)
	{
		typedef PointMatcher<T> PM;
		typedef typename PM::DataPoints DataPoints;
		typedef typename DataPoints::Label Label;
		typedef typename DataPoints::Labels Labels;
		typedef typename DataPoints::View View;
		
		if (rosMsg.fields.empty())
			return DataPoints();
		
		// fill labels
		Labels featLabels;
		Labels descLabels;
		vector<bool> isFeature(rosMsg.fields.size());
		size_t fieldId(0);
		for(auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it, ++fieldId)
		{
			const string name(it->name);
			const size_t count(std::max<size_t>(it->count, 1));
			if (name == "x" || name == "y" || name == "z")
			{
				featLabels.push_back(Label(name, count));
				isFeature[fieldId] = true;
			}
			else
			{
				descLabels.push_back(Label(name, count));
				isFeature[fieldId] = false;
			}
		}
		featLabels.push_back(Label("pad", 1));
		
		// create cloud
		const unsigned pointCount(rosMsg.width * rosMsg.height);
		DataPoints cloud(featLabels, descLabels, pointCount);
		
		// fill cloud
		// TODO: support big endian, pass through endian-swapping method just after the *reinterpret_cast
		typedef sensor_msgs::PointField PF;
		fieldId = 0;
		for(auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it, ++fieldId)
		{
			View view(
				isFeature[fieldId] ?
				cloud.getFeatureViewByName(it->name) :
				cloud.getDescriptorViewByName(it->name)
			);
			int ptId(0);
			const size_t count(std::max<size_t>(it->count, 1));
			for (size_t y(0); y < rosMsg.height; ++y)
			{
				const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step*y);
				for (size_t x(0); x < rosMsg.width; ++x)
				{
					const uint8_t* fPtr(dataPtr + it->offset);
					for (unsigned dim(0); dim < count; ++dim)
					{
						switch (it->datatype)
						{
							case PF::INT8: view(dim, ptId) = T(*reinterpret_cast<const int8_t*>(fPtr)); fPtr += 1; break;
							case PF::UINT8: view(dim, ptId) = T(*reinterpret_cast<const uint8_t*>(fPtr)); fPtr += 1; break;
							case PF::INT16: view(dim, ptId) = T(*reinterpret_cast<const int16_t*>(fPtr)); fPtr += 2; break;
							case PF::UINT16: view(dim, ptId) = T(*reinterpret_cast<const uint16_t*>(fPtr)); fPtr += 2; break;
							case PF::INT32: view(dim, ptId) = T(*reinterpret_cast<const int32_t*>(fPtr)); fPtr += 4; break;
							case PF::UINT32: view(dim, ptId) = T(*reinterpret_cast<const uint32_t*>(fPtr)); fPtr += 4; break;
							case PF::FLOAT32: view(dim, ptId) = T(*reinterpret_cast<const float*>(fPtr)); fPtr += 4; break;
							case PF::FLOAT64: view(dim, ptId) = T(*reinterpret_cast<const double*>(fPtr)); fPtr += 8; break;
							default: abort();
						}
					}
					dataPtr += rosMsg.point_step;
					ptId += 1;
				}
			}
		}
		
		shared_ptr<typename PM::DataPointsFilter> filter(PM::get().DataPointsFilterRegistrar.create("RemoveNaNDataPointsFilter"));
		return filter->filter(cloud);
	}
	
	template
	typename PointMatcher<float>::DataPoints rosMsgToPointMatcherCloud<float>(const sensor_msgs::PointCloud2& rosMsg);
	template
	typename PointMatcher<double>::DataPoints rosMsgToPointMatcherCloud<double>(const sensor_msgs::PointCloud2& rosMsg);


	template<typename T>
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp)
	{
		sensor_msgs::PointCloud2 rosCloud;
		
		// check type and get sizes
		BOOST_STATIC_ASSERT(is_floating_point<T>::value);
		BOOST_STATIC_ASSERT((is_same<T, long double>::value == false));
		uint8_t dataType;
		size_t scalarSize;
		if (typeid(T) == typeid(float))
		{
			dataType = 7;
			scalarSize = 4;
		}
		else
		{
			dataType = 8;
			scalarSize = 8;
		}
		
		// build labels
		typedef sensor_msgs::PointField PF;
		unsigned offset(0);
		assert(!pmCloud.featureLabels.empty());
		assert(pmCloud.featureLabels[pmCloud.featureLabels.size()-1].text == "pad");
		for(auto it(pmCloud.featureLabels.begin()); it != pmCloud.featureLabels.end(); ++it)
		{
			// last label is padding
			if ((it + 1) == pmCloud.featureLabels.end())
				break;
			PF pointField;
			pointField.name = it->text;
			pointField.offset = offset;
			pointField.datatype= dataType;
			pointField.count = it->span;
			rosCloud.fields.push_back(pointField);
			offset += it->span * scalarSize;
		}
		for(auto it(pmCloud.descriptorLabels.begin()); it != pmCloud.descriptorLabels.end(); ++it)
		{
			PF pointField;
			pointField.name = it->text;
			pointField.offset = offset;
			pointField.datatype= dataType;
			pointField.count = it->span;
			rosCloud.fields.push_back(pointField);
			offset += it->span * scalarSize;
		}
		
		// fill cloud with data
		rosCloud.header.frame_id = frame_id;
		rosCloud.header.stamp = stamp;
		rosCloud.height = 1;
		rosCloud.width = pmCloud.features.cols();
		#ifdef BOOST_BIG_ENDIAN
		rosCloud.is_bigendian = true;
		#else // BOOST_BIG_ENDIAN
		rosCloud.is_bigendian = false;
		#endif // BOOST_BIG_ENDIAN
		rosCloud.point_step = offset;
		rosCloud.row_step = rosCloud.point_step * rosCloud.width;
		rosCloud.is_dense = true;
		rosCloud.data.resize(rosCloud.row_step * rosCloud.height);
		const unsigned featureDim(pmCloud.features.rows()-1);
		const unsigned descriptorDim(pmCloud.descriptors.rows());
		for (unsigned pt(0); pt < rosCloud.width; ++pt)
		{
			uint8_t *fPtr(&rosCloud.data[pt * offset]);
			memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.features(0, pt)), scalarSize * featureDim);
			memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(0, pt)), scalarSize * descriptorDim);
		}

		// fill remaining information
		rosCloud.header.frame_id = frame_id;
		rosCloud.header.stamp = stamp;
		
		return rosCloud;
	}
	
	template
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg<float>(const typename PointMatcher<float>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);
	template
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg<double>(const typename PointMatcher<double>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);

} // PointMatcher_ros
