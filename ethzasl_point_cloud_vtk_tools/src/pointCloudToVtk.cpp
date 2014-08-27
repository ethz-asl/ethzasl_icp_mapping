#include <fstream>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "pointmatcher/PointMatcher.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"

using namespace std;
using namespace PointMatcherSupport;

class ExportVtk
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    ros::NodeHandle& n;

    ros::Subscriber cloudSub;
    const string cloudTopic;
    const string mapFrame;
    const bool recordOnce;
    const string outputPath;
    string outputPrefix;
    string outputExtension;

    tf::TransformListener tf_listener;
    std::shared_ptr<PM::Transformation> transformation;

    void validateOutputExtension();

public:
    ExportVtk(ros::NodeHandle& n);
    void gotCloud(const sensor_msgs::PointCloud2& cloudMsg);
};

ExportVtk::ExportVtk(ros::NodeHandle& n):
    n(n),
    cloudTopic(getParam<string>("cloudTopic", "/static_point_cloud")),
    mapFrame(getParam<string>("mapFrameId", "/map")),
    recordOnce(getParam<bool>("recordOnce", "false")),
    outputPath(getParam<string>("outputPath", "")),
    outputPrefix(getParam<string>("outputPrefix", "")),
    outputExtension(getParam<string>("outputExtension", ".vtk")),
    transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
{
    if(outputPrefix.empty())
    {
        outputPrefix = cloudTopic;
    }
    validateOutputExtension();

    // ROS initialization
    cloudSub = n.subscribe(cloudTopic, 100, &ExportVtk::gotCloud, this);

    // Parameters for 3D map
}

void ExportVtk::validateOutputExtension()
{
    if(!outputExtension.empty() && outputExtension.at(0) != '.')
    {
        outputExtension = "." + outputExtension;
    }

    std::locale loc;
    for(std::string::size_type i=0; i < outputExtension.length(); ++i)
    {
        outputExtension[i] = std::tolower(outputExtension[i], loc);
    }

    if(outputExtension.compare(".csv") != 0 &&
            outputExtension.compare(".vtk") != 0 &&
            outputExtension.compare(".ply") != 0 &&
            outputExtension.compare(".pcd") != 0)
    {
        outputExtension = ".vtk";
    }
}


void ExportVtk::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    const DP inCloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    try
    {
        const PM::TransformationParameters tr(PointMatcher_ros::transformListenerToEigenMatrix<float>(tf_listener,  mapFrame, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp));

        const DP outCloud(transformation->compute(inCloud, tr));

        if (outCloud.features.cols() == 0)
        {
            ROS_ERROR("I found no good points in the cloud");
            return;
        }
        else
        {
            ROS_INFO("Saving cloud");
        }

        stringstream nameStream;
        if(outputPath.empty())
        {
            nameStream << ".";
        }
        else
        {
            nameStream << outputPath;
        }
        nameStream << outputPrefix << "_" << cloudMsgIn.header.seq << outputExtension;

        outCloud.save(nameStream.str());
        if(recordOnce)
        {
            ros::shutdown();
        }
    }
    catch (tf::ExtrapolationException e)
    {
        ROS_WARN("Still build tf tree, skipping that scan");
    }
}

// Main function supporting the ExportVtk class
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointCloudToVtk_node");
    ros::NodeHandle n;
    ExportVtk exporter(n);
    ros::spin();

    // Wait for the shutdown to finish
    while(ros::ok()){}

    return 0;
}
