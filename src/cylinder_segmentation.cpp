/* Detects cylinder from PointCloud data */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <vector>

// Set the type of pointcloud here
typedef pcl::PointXYZ PointT;


class CylinderFinder
{
private:
    // ROS Objects
    ros::NodeHandle nh;
    ros::Subscriber raw_pc_sub; // subscribe to raw pointcloud data
    ros::Publisher seg_pc_pub; // publish the pointcloud after segmentation
    ros::Publisher filtered_pc_pub; // publish the filtered pointcloud for debugging

    // PCL Objects
    pcl::PointCloud<PointT>::Ptr cloud;
    pcl::PointCloud<PointT>::Ptr cloud_filtered;
    pcl::PointCloud<PointT>::Ptr cloud_filtered2;
    pcl::PointCloud<PointT>::Ptr cloud_cylinder;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2;
    pcl::PassThrough<PointT> pass; // passthrough filter
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract; // another filter
    pcl::ExtractIndices<pcl::Normal> extract_normals; // filter for normals
    pcl::search::KdTree<PointT>::Ptr tree;//(new pcl::search::KdTree<PointT>());
    pcl::ModelCoefficients::Ptr coefficients_cylinder, coefficients_plane;
    pcl::PointIndices::Ptr inliers_cylinder, inliers_plane;

public:
    CylinderFinder() :
    nh("~"),
    cloud(new pcl::PointCloud<PointT>),
    cloud_filtered(new pcl::PointCloud<PointT>),
    cloud_filtered2(new pcl::PointCloud<PointT>),
    cloud_cylinder(new pcl::PointCloud<PointT>),
    cloud_normals(new pcl::PointCloud<pcl::Normal>),
    cloud_normals2(new pcl::PointCloud<pcl::Normal>),
    coefficients_cylinder(new pcl::ModelCoefficients),
    coefficients_plane(new pcl::ModelCoefficients),
    inliers_cylinder(new pcl::PointIndices),
    inliers_plane(new pcl::PointIndices),
    tree(new pcl::search::KdTree<PointT>)
    {
        raw_pc_sub = nh.subscribe("/camera/depth/points", 1, &CylinderFinder::pcCallback, this);
        //raw_pc_sub = nh.subscribe("/pcd_publisher/points", 1, &CylinderFinder::pcCallback, this);
        seg_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("points_segmented", 1);
        filtered_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("points_filtered", 1);
    }

    void pcCallback(const sensor_msgs::PointCloud2 &msg)
    {
        // Save header
        std_msgs::Header header = msg.header;

        // Convert into PCL type
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        //*** Cylinder Model Segmentation Example ***//
        //(http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php#cylinder-segmentation)
        // This code has been ROS-ified

        // Filter out 'NaN's
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 2.0); // make this value big enough to include all points
        pass.filter(*cloud_filtered);

        // Estimate point normals
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_filtered);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        // Segment out Planar surfaces to remove them
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight(0.1);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.15);
        seg.setInputCloud(cloud_filtered);
        seg.setInputNormals(cloud_normals);
        seg.segment(*inliers_plane, *coefficients_plane);
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers_plane);
        extract.setNegative(true); // this removes the inliers and keeps everything else (so it removes the planar points)
        extract.filter(*cloud_filtered2);
        extract_normals.setInputCloud(cloud_normals);
        extract_normals.setIndices(inliers_plane);
        extract_normals.setNegative(true);
        extract_normals.filter(*cloud_normals2);

        // Create segmentation object for cylinder model and set parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.01);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(1.0);
        seg.setRadiusLimits(0, 0.2);
        seg.setInputCloud(cloud_filtered2);
        seg.setInputNormals(cloud_normals2);

        // Find cylinder inliers and coefficients
        seg.segment(*inliers_cylinder, *coefficients_cylinder);

        // // FIXME: print out cylinder coefficients
        // if (inliers_cylinder->indices.size() > 0) {
        //     std::cout << "Cylinder coefficients: (" << coefficients_cylinder->values[0] << ", " << coefficients_cylinder->values[1] << ", " << coefficients_cylinder->values[2] << ", " << coefficients_cylinder->values[3] << ", " << coefficients_cylinder->values[4] << ", " << coefficients_cylinder->values[5] << ", " << coefficients_cylinder->values[6] << ")" << std::endl;
        // }

        // Extract cylinder points to a cloud
        extract.setInputCloud(cloud_filtered2);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        extract.filter(*cloud_cylinder);
        //*** Cylinder Model Segmentation Example ***//

        // Convert to ROS PointCloud2 message
        sensor_msgs::PointCloud2 pointCloudMsg;
        pcl::toPCLPointCloud2(*cloud_cylinder, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, pointCloudMsg);

        // DEBUG: create message of filtered cloud for debugging
        sensor_msgs::PointCloud2 filteredCloudMsg;
        pcl::toPCLPointCloud2(*cloud_filtered2, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, filteredCloudMsg);
        filteredCloudMsg.header = header;

        // Copy header data over
        pointCloudMsg.header = header;

        // Publish
        seg_pc_pub.publish(pointCloudMsg);
        filtered_pc_pub.publish(filteredCloudMsg);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cylinder_segmentation");
    CylinderFinder cylFind;
    ros::spin();

    return 0;
}
