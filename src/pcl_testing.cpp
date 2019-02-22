// PCL Testing code, to figure out how things work

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

pcl::visualization::CloudViewer viewer("Simple Viewer");

void pcCallback(const sensor_msgs::PointCloud2& msg)
{
    // Display the data in a window
    pcl::PCLPointCloud2 pcl_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_conversions::toPCL(msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);

    // Perform passthrough filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(pointCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.5, 0.5);
    pass.filter(*pointCloud_filtered);

    // Perform voxel grid filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(pointCloud_filtered);
    sor.setLeafSize(0.1f, 0.01f, 0.1f); // larger numbers = less sample points
    sor.filter(*pointCloud_voxel);

    viewer.showCloud(pointCloud_voxel);
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
