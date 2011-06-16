#ifndef __CLOUD_CONVERSION_H
#define __CLOUD_CONVERSION_H

#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h" 
#include "pcl/ros/conversions.h"

DP rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg, size_t& goodCount)
{
	goodCount = 0;
	
	// create labels
	DP::Labels labels;
	labels.push_back(DP::Label("x", 1));
	labels.push_back(DP::Label("y", 1));
	labels.push_back(DP::Label("z", 1));
	labels.push_back(DP::Label("pad", 1));
	
	// create data points
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (rosMsg, cloud);
	
	// scan points for goods
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		if (!isnan(cloud.points[i].x))
			++goodCount;
	}
	
	// do not do anything if all points are bad
	if (goodCount == 0)
		return DP();
	
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
	return dp;
}

#endif
