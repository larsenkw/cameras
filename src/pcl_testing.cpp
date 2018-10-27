// PCL Testing code, to figure out how things work

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::visualization::CloudViewer viewer("Simple Viewer");

void pcCallback(const sensor_msgs::PointCloud2& msg)
{
    // Display the data in a window
    pcl::PCLPointCloud2 pcl_pc;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_conversions::toPCL(msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
    viewer.showCloud(pointCloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_testing");
    ros::NodeHandle nh;

    // Subscribe to point cloud data from Astra Pro
    //ros::Subscriber pc_sub = nh.subscribe("/astrapro/depth/points", 1, pcCallback);
    //ros::Subscriber pc_sub = nh.subscribe("/astra/depth_registered/points", 1, pcCallback);
    ros::Subscriber pc_sub = nh.subscribe("/os1_node/points", 1, pcCallback);

    ros::spin();

    return 0;
}
