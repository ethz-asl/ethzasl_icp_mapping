#include "pointmatcher_ros/point_cloud.h"
#include "ros/ros.h"
#include "boost/detail/endian.hpp"
#include "tf/transform_listener.h"
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
		// conversions of descriptor fields from pcl
		// see http://www.ros.org/wiki/pcl/Overview
		Labels featLabels;
		Labels descLabels;
		vector<bool> isFeature;
		for(auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it)
		{
			const string name(it->name);
			const size_t count(std::max<size_t>(it->count, 1));
			if (name == "x" || name == "y" || name == "z")
			{
				featLabels.push_back(Label(name, count));
				isFeature.push_back(true);
			}
			else if (name == "rgb" || name == "rgba")
			{
				descLabels.push_back(Label("color", (name == "rgba") ? 4 : 3));
				isFeature.push_back(false);
			}
			else if ((it+1) != rosMsg.fields.end() && it->name == "normal_x" && (it+1)->name == "normal_y")
			{
				if ((it+2) != rosMsg.fields.end() && (it+2)->name == "normal_z")
				{
					descLabels.push_back(Label("normals", 3));
					it += 2;
					isFeature.push_back(false);
					isFeature.push_back(false);
				}
				else
				{
					descLabels.push_back(Label("normals", 2));
					it += 1;
					isFeature.push_back(false);
				}
				isFeature.push_back(false);
			}
			else 
			{
				descLabels.push_back(Label(name, count));
				isFeature.push_back(false);
			}
		}
		featLabels.push_back(Label("pad", 1));
		assert(isFeature.size() == rosMsg.fields.size());
		
		// create cloud
		const unsigned pointCount(rosMsg.width * rosMsg.height);
		DataPoints cloud(featLabels, descLabels, pointCount);
		cloud.getFeatureViewByName("pad").setConstant(1);
		
		// fill cloud
		// TODO: support big endian, pass through endian-swapping method just after the *reinterpret_cast
		typedef sensor_msgs::PointField PF;
		size_t fieldId = 0;
		for(auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it, ++fieldId)
		{
			if (it->name == "rgb" || it->name == "rgba")
			{
				// special case for colors
				if (((it->datatype != PF::UINT32) && (it->datatype != PF::INT32) && (it->datatype != PF::FLOAT32)) || (it->count != 1))
					throw runtime_error(
						(boost::format("Colors in a point cloud must be a single element of size 32 bits, found %1% elements of type %2%") % it->count % unsigned(it->datatype)).str()
					);
				View view(cloud.getDescriptorViewByName("color"));
				int ptId(0);
				for (size_t y(0); y < rosMsg.height; ++y)
				{
					const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step*y);
					for (size_t x(0); x < rosMsg.width; ++x)
					{
						const uint32_t rgba(*reinterpret_cast<const uint32_t*>(dataPtr + it->offset));
						const T colorA(T((rgba >> 24) & 0xff) / 255.);
						const T colorR(T((rgba >> 16) & 0xff) / 255.);
						const T colorG(T((rgba >> 8) & 0xff) / 255.);
						const T colorB(T((rgba >> 0) & 0xff) / 255.);
						view(0, ptId) = colorR;
						view(1, ptId) = colorG;
						view(2, ptId) = colorB;
						if (view.rows() > 3)
							view(3, ptId) = colorA;
						dataPtr += rosMsg.point_step;
						ptId += 1;
					}
				}
			}
			else
			{
				// get view for editing data
				View view(
					 (it->name == "normal_x") ? cloud.getDescriptorRowViewByName("normals", 0) :
					((it->name == "normal_y") ? cloud.getDescriptorRowViewByName("normals", 1) :
					((it->name == "normal_z") ? cloud.getDescriptorRowViewByName("normals", 2) :
					((isFeature[fieldId]) ? cloud.getFeatureViewByName(it->name) :
					cloud.getDescriptorViewByName(it->name))))
				);
				// use view to read data
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
		}

		
		shared_ptr<typename PM::DataPointsFilter> filter(PM::get().DataPointsFilterRegistrar.create("RemoveNaNDataPointsFilter"));
		return filter->filter(cloud);
	}
	
	template
	PointMatcher<float>::DataPoints rosMsgToPointMatcherCloud<float>(const sensor_msgs::PointCloud2& rosMsg);
	template
	PointMatcher<double>::DataPoints rosMsgToPointMatcherCloud<double>(const sensor_msgs::PointCloud2& rosMsg);
	
	
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::LaserScan& rosMsg, const tf::TransformListener* listener, const std::string& fixedFrame, const bool force3D, const bool addTimestamps, const bool addObservationDirection)
	{
		typedef PointMatcher<T> PM;
		typedef typename PM::DataPoints DataPoints;
		typedef typename DataPoints::Label Label;
		typedef typename DataPoints::Labels Labels;
		typedef typename DataPoints::View View;
		
		Labels featLabels;
		featLabels.push_back(Label("x", 1));
		featLabels.push_back(Label("y", 1));
		if(force3D)
			featLabels.push_back(Label("z", 1));

		featLabels.push_back(Label("pad", 1));
		
    // Build descriptors
		Labels descLabels;
		if (!rosMsg.intensities.empty())
		{
			descLabels.push_back(Label("intensity", 1));
			assert(rosMsg.intensities.size() == rosMsg.ranges.size());
		}
    
    int id_obs = 0;
    if(addObservationDirection)
    {
			descLabels.push_back(Label("observationDirections", 3));
    }
		
    //TODO: change that once the time_support branch is merge in libpointmatcher
    if(addTimestamps)
    {
			descLabels.push_back(Label("timestamp", 3));
    }
		// filter points based on range
    std::vector<size_t> ids(rosMsg.ranges.size());
    std::vector<double> ranges(rosMsg.ranges.size());
    std::vector<double> intensities(rosMsg.intensities.size());

		size_t goodCount(0);
		for (size_t i = 0; i < rosMsg.ranges.size(); ++i)
		{
			const float range(rosMsg.ranges[i]);
			if (range >= rosMsg.range_min && range <= rosMsg.range_max)
      {
        ranges[goodCount] = range;
        ids[goodCount] = i;
        if(!rosMsg.intensities.empty())
        {
          intensities[goodCount] = rosMsg.intensities[i];
        }
				++goodCount;
      }
		}
		if (goodCount == 0)
			return DataPoints();

    ids.resize(goodCount);
    ranges.resize(goodCount);
    if(!rosMsg.intensities.empty())
      intensities.resize(goodCount);

		DataPoints cloud(featLabels, descLabels, goodCount);
		cloud.getFeatureViewByName("pad").setConstant(1);
      
    if(addObservationDirection)
    {
      id_obs = cloud.getDescriptorStartingRow("observationDirections");
    }
		
		// fill features
		const ros::Time& startTime(rosMsg.header.stamp);
		const ros::Time endTime(startTime + ros::Duration(rosMsg.time_increment * (rosMsg.ranges.size() - 1)));
    
		for (size_t i = 0; i < ranges.size(); ++i)
		{
			const T angle = rosMsg.angle_min + ids[i]*rosMsg.angle_increment;
			const T range(ranges[i]);
      const T x = cos(angle) * range;
      const T y = sin(angle) * range;

      // the turn ratio correct for the fact that not all sensor scan
      // continuously during 360 deg 
      const float turnRatio = (rosMsg.angle_max - rosMsg.angle_min)/(2*M_PI);
      // dt_point should be more precise than rosMsg.time_increment
      const float dt_point = (rosMsg.scan_time*turnRatio)/rosMsg.ranges.size();

      if (listener)
      {
        
        const ros::Time curTime(rosMsg.header.stamp + ros::Duration(ids[i] * dt_point));

        // wait for transform
        listener->waitForTransform(
          rosMsg.header.frame_id,
          fixedFrame,
          curTime,
          ros::Duration(0.1)
        );

        // transform point
        geometry_msgs::PointStamped pin, p_out;
        pin.header.stamp = curTime;
        pin.header.frame_id = rosMsg.header.frame_id;
        pin.point.x = x;
        pin.point.y = y;
        pin.point.z = 0;
        
        // transform sensor center
        geometry_msgs::PointStamped s_in, s_out;
        s_in.header.stamp = curTime;
        s_in.header.frame_id = rosMsg.header.frame_id;
        s_in.point.x = 0;
        s_in.point.y = 0;
        s_in.point.z = 0;

        try
        {
      
          listener->transformPoint(
						fixedFrame,
            rosMsg.header.stamp,
            pin,
            fixedFrame,
            p_out
          );

          if(addObservationDirection)
          {
            listener->transformPoint(
                fixedFrame,
                curTime,
                s_in,
                fixedFrame,
                s_out
                );
          }
        }
        catch (const tf::ExtrapolationException& e)
        {
          //ROS_WARN_STREAM("libpointmatcher_ros: Couldn't transform point: " << e.what());
          return DataPoints();
        }

        //cout << "pin: " << pin.point.x << ", " << pin.point.y << ", " << pin.point.z << endl;
        //cout << "p_out: " << p_out.point.x << ", " << p_out.point.y << ", " << p_out.point.z << endl;

        // write back
        cloud.features(0,i) = p_out.point.x;
        cloud.features(1,i) = p_out.point.y;
        if(force3D)
          cloud.features(2,i) = p_out.point.z;
				
        if(addObservationDirection)
        {
          cloud.descriptors(id_obs  , i) = s_out.point.x - p_out.point.x;
          cloud.descriptors(id_obs+1, i) = s_out.point.y - p_out.point.y;
          cloud.descriptors(id_obs+2, i) = s_out.point.z - p_out.point.z;
        }
			}
		}

		// fill descriptors
		if (!rosMsg.intensities.empty())
		{
			auto is(cloud.getDescriptorViewByName("intensity"));
			for (size_t i = 0; i < intensities.size(); ++i)
			{
					is(0,i) = intensities[i];
			}
		}

    if(addTimestamps)
    {
			auto is(cloud.getDescriptorViewByName("timestamp"));

			for (size_t i = 0; i < ranges.size(); ++i)
      {
        const ros::Time curTime(rosMsg.header.stamp + ros::Duration(ids[i] * rosMsg.time_increment));

        const T Msec = round(curTime.sec/1e6);
        const T sec  = round(curTime.sec - Msec*1e6);
        const T nsec = round(curTime.nsec);

        is(0,i) = Msec;
        is(1,i) = sec;
        is(2,i) = nsec;
      }
    }
		

		//cerr << "point cloud:\n" << cloud.features.leftCols(10) << endl;
		return cloud;
	}
	
	template
	PointMatcher<float>::DataPoints rosMsgToPointMatcherCloud<float>(const sensor_msgs::LaserScan& rosMsg, const tf::TransformListener* listener, const std::string& fixedFrame, const bool force3D, const bool addTimestamps, const bool addObservationDirection);
	template
	PointMatcher<double>::DataPoints rosMsgToPointMatcherCloud<double>(const sensor_msgs::LaserScan& rosMsg, const tf::TransformListener* listener, const std::string& fixedFrame, const bool force3D, const bool addTimestamps, const bool addObservationDirection);


	template<typename T>
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp)
	{
		sensor_msgs::PointCloud2 rosCloud;
		typedef sensor_msgs::PointField PF;
		
		// check type and get sizes
		BOOST_STATIC_ASSERT(is_floating_point<T>::value);
		BOOST_STATIC_ASSERT((is_same<T, long double>::value == false));
		uint8_t dataType;
		size_t scalarSize;
		if (typeid(T) == typeid(float))
		{
			dataType = PF::FLOAT32;
			scalarSize = 4;
		}
		else
		{
			dataType = PF::FLOAT64;
			scalarSize = 8;
		}
		
		// build labels
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
		bool addZ(false);
		if (!pmCloud.featureLabels.contains("z"))
		{
			PF pointField;
			pointField.name = "z";
			pointField.offset = offset;
			pointField.datatype= dataType;
			pointField.count = 1;
			rosCloud.fields.push_back(pointField);
			offset += scalarSize;
			addZ = true;
		}
		const bool isDescriptor(!pmCloud.descriptorLabels.empty());
		bool hasColor(false);
		unsigned colorPos(0);
		unsigned colorCount(0);
		unsigned inDescriptorPos(0);
		for(auto it(pmCloud.descriptorLabels.begin()); it != pmCloud.descriptorLabels.end(); ++it)
		{
			PF pointField;
			if (it->text == "normals")
			{
				assert((it->span == 2) || (it->span == 3));
				pointField.datatype = dataType;
				pointField.name = "normal_x";
				pointField.offset = offset;
				pointField.count = 1;
				rosCloud.fields.push_back(pointField);
				offset += scalarSize;
				pointField.name = "normal_y";
				pointField.offset = offset;
				pointField.count = 1;
				rosCloud.fields.push_back(pointField);
				offset += scalarSize;
				if (it->span == 3)
				{
					pointField.name = "normal_z";
					pointField.offset = offset;
					pointField.count = 1;
					rosCloud.fields.push_back(pointField);
					offset += scalarSize;
				}
			}
			else if (it->text == "color")
			{
				colorPos = inDescriptorPos;
				colorCount = it->span;
				hasColor = true;
				pointField.datatype = (colorCount == 4) ? uint8_t(PF::UINT32) : uint8_t(PF::FLOAT32);
				pointField.name = (colorCount == 4) ? "rgba" : "rgb";
				pointField.offset = offset;
				pointField.count = 1;
				rosCloud.fields.push_back(pointField);
				offset += 4;
			}
			else
			{
				pointField.datatype = dataType;
				pointField.name = it->text;
				pointField.offset = offset;
				pointField.count = it->span;
				rosCloud.fields.push_back(pointField);
				offset += it->span * scalarSize;
			}
			inDescriptorPos += it->span;
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
		assert(descriptorDim == inDescriptorPos);
		const unsigned postColorPos(colorPos + colorCount);
		assert(postColorPos <= inDescriptorPos);
		const unsigned postColorCount(descriptorDim - postColorPos);
		for (unsigned pt(0); pt < rosCloud.width; ++pt)
		{
			uint8_t *fPtr(&rosCloud.data[pt * offset]);
			memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.features(0, pt)), scalarSize * featureDim);
			fPtr += scalarSize * featureDim;
			if (addZ)
			{
				memset(fPtr, 0, scalarSize);
				fPtr += scalarSize;
			}
			if (isDescriptor)
			{
				if (hasColor)
				{
					// before color
					memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(0, pt)), scalarSize * colorPos);
					fPtr += scalarSize * colorPos;
					// compact color
					uint32_t rgba;
					unsigned colorR(unsigned(pmCloud.descriptors(colorPos+0, pt) * 255.) & 0xFF);
					unsigned colorG(unsigned(pmCloud.descriptors(colorPos+1, pt) * 255.) & 0xFF);
					unsigned colorB(unsigned(pmCloud.descriptors(colorPos+2, pt) * 255.) & 0xFF);
					unsigned colorA(0);
					if (colorCount == 4)
						colorA = unsigned(pmCloud.descriptors(colorPos+3, pt) * 255.) & 0xFF;
					rgba = colorA << 24 | colorR << 16 | colorG << 8 | colorB;
					memcpy(fPtr, reinterpret_cast<const uint8_t*>(&rgba), 4);
					fPtr += 4;
					// after color
					memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(postColorPos, pt)), scalarSize * postColorCount);
				}
				else
				{
					memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(0, pt)), scalarSize * descriptorDim);
				}
			}
		}

		// fill remaining information
		rosCloud.header.frame_id = frame_id;
		rosCloud.header.stamp = stamp;
		
		return rosCloud;
	}
	
	template
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg<float>(const PointMatcher<float>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);
	template
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg<double>(const PointMatcher<double>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);

} // PointMatcher_ros
