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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <vector>

// Set the type of pointcloud here
typedef pcl::PointXYZ PointT;

// // DEBUG: PCL Visualizer for debugging
// pcl::visualization::PCLVisualizer viewer("Debugging");

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
    pcl::PointCloud<PointT>::Ptr cloud_passthrough;
    pcl::PointCloud<PointT>::Ptr cloud_filtered;
    pcl::PointCloud<PointT>::Ptr cloud_filtered2;
    pcl::PointCloud<PointT>::Ptr cloud_cylinder;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2;
    pcl::PassThrough<PointT> pass; // passthrough filter
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::NormalEstimation<PointT, pcl::Normal> ne_mls;
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
    cloud_passthrough(new pcl::PointCloud<PointT>),
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

        // Use passthrough filter to remove data out of range and to remove 'Nan's
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.5); // most reliable data is <6m
        pass.filter(*cloud_passthrough);

        // Filter out noise
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud_passthrough);
        sor.setMeanK(10);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_filtered);

        // Downsample with VoxelGrid
        pcl::VoxelGrid<PointT> voxel;
        pcl::PointCloud<PointT>::Ptr cloud_downsample(new pcl::PointCloud<PointT>);
        voxel.setInputCloud(cloud_filtered);
        voxel.setLeafSize(0.01f, 0.01f, 0.01f);
        voxel.filter(*cloud_downsample);

        // Estimate point normals
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_downsample);
        ne.setKSearch(10);
        ne.compute(*cloud_normals);

        // Segment out Planar surfaces to remove them
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight(0.1);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.15);
        seg.setInputCloud(cloud_downsample);
        seg.setInputNormals(cloud_normals);
        seg.segment(*inliers_plane, *coefficients_plane);
        extract.setInputCloud(cloud_downsample);
        extract.setIndices(inliers_plane);
        extract.setNegative(true); // this removes the inliers and keeps everything else (so it removes the planar points)
        extract.filter(*cloud_filtered2);
        extract_normals.setInputCloud(cloud_normals);
        extract_normals.setIndices(inliers_plane);
        extract_normals.setNegative(true);
        extract_normals.filter(*cloud_normals2);

        // Smooth the cloud with Moving Least Squares(MLS)
        pcl::PointCloud<PointT>::Ptr cloud_mls(new pcl::PointCloud<PointT>);
        pcl::MovingLeastSquares<PointT, PointT> mls;
        mls.setInputCloud(cloud_filtered2);
        mls.setPolynomialFit(true);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(0.03);
        mls.process(*cloud_mls);

        // Remove NaNs if any
        std::vector<int> indices;
        pcl::PointCloud<PointT>::Ptr nan_removed(new pcl::PointCloud<PointT>);
        int nan_removed_index = 0;
        for (size_t i = 0; i < cloud_mls->points.size(); ++i) {
            if (!isnan(cloud_mls->points[i].x)) {
                nan_removed->resize(nan_removed_index+1);
                nan_removed->points[nan_removed_index].x = cloud_mls->points[i].x;
                nan_removed->points[nan_removed_index].y = cloud_mls->points[i].y;
                nan_removed->points[nan_removed_index].z = cloud_mls->points[i].z;
                nan_removed_index++;
            }
        }
        pcl::copyPointCloud(*nan_removed, *cloud_mls);

        // Estimate point normals
        pcl::PointCloud<pcl::Normal>::Ptr mls_normals(new pcl::PointCloud<pcl::Normal>);
        ne_mls.setSearchMethod(tree);
        ne_mls.setInputCloud(cloud_mls);
        ne_mls.setKSearch(10);
        ne_mls.compute(*mls_normals);

        // Create segmentation object for cylinder model and set parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.01);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.1);
        seg.setRadiusLimits(0.0, 0.1);
        seg.setInputCloud(cloud_mls);
        seg.setInputNormals(mls_normals);

        // Find cylinder inliers and coefficients
        seg.segment(*inliers_cylinder, *coefficients_cylinder);

        // // DEBUG: view point cloud normals
        // viewer.removePointCloud("normals");
        // viewer.addPointCloudNormals<PointT, pcl::Normal>(cloud_mls, mls_normals, 1, 0.02, "normals");
        // viewer.spinOnce(100);

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
        pcl::toPCLPointCloud2(*cloud_mls, pcl_pc2);
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
