/* Detects cylinder from PointCloud data */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/pcl_config.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <vector>

// Set the type of pointcloud here
typedef pcl::PointXYZ PointT;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;



// DEBUG: PCL Visualizer for debugging
pcl::visualization::PCLVisualizer viewer("Debugging");

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

    std::string model_file;

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

        model_file = "/home/csm/forklift_ws/src/cameras/models/poster_cylinder.pcd";
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

        // // FIXME: save scene to file for testing purposes
        // pcl::io::savePCDFileASCII("cylinder_test_scene.pcd", *cloud);

        //*** Cylinder Model Segmentation Example ***//
        //(http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php#cylinder-segmentation)
        // This code has been ROS-ified

        // Use passthrough filter to remove data out of range and to remove 'Nan's
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 5); // most reliable data is <6m
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
        pcl::search::KdTree<PointT>::Ptr tree2(new pcl::search::KdTree<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_mls(new pcl::PointCloud<PointT>);
        pcl::MovingLeastSquares<PointT, PointT> mls;
        mls.setInputCloud(cloud_filtered2);
        mls.setPolynomialFit(true);
        mls.setSearchMethod(tree2);
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

        // // Create segmentation object for cylinder model and set parameters
        // seg.setOptimizeCoefficients(true);
        // seg.setModelType(pcl::SACMODEL_CYLINDER);
        // seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setNormalDistanceWeight(0.01);
        // seg.setMaxIterations(10000);
        // seg.setDistanceThreshold(0.1);
        // seg.setRadiusLimits(0.0, 0.1);
        // seg.setInputCloud(cloud_mls);
        // seg.setInputNormals(mls_normals);

        // // Find cylinder inliers and coefficients
        // seg.segment(*inliers_cylinder, *coefficients_cylinder);

        // // FIXME: print out cylinder coefficients
        // if (inliers_cylinder->indices.size() > 0) {
        //     std::cout << "Cylinder coefficients: (" << coefficients_cylinder->values[0] << ", " << coefficients_cylinder->values[1] << ", " << coefficients_cylinder->values[2] << ", " << coefficients_cylinder->values[3] << ", " << coefficients_cylinder->values[4] << ", " << coefficients_cylinder->values[5] << ", " << coefficients_cylinder->values[6] << ")" << std::endl;
        // }

        // // Extract cylinder points to a cloud
        // extract.setInputCloud(cloud_filtered2);
        // extract.setIndices(inliers_cylinder);
        // extract.setNegative(false);
        // extract.filter(*cloud_cylinder);
        // //*** Cylinder Model Segmentation Example ***//

        //==================================================================
        // Try using the 3D object recognition using correspondence grouping
        //==================================================================
        //Algorithm params
        bool show_keypoints_ (false);
        bool show_correspondences_ (false);
        bool use_cloud_resolution_ (false);
        bool use_hough_ (true);
        float model_ss_ (0.03f);
        float scene_ss_ (0.03f);
        float rf_rad_ (0.015f);
        float descr_rad_ (0.02f);
        float cg_size_ (0.01f);
        float cg_thresh_ (5.0f);

        // Load model
        pcl::PointCloud<PointT>::Ptr model(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile(model_file.c_str(), *model) < 0) {
            ROS_INFO("Unable to load model file: %s", model_file.c_str());
        }

        // Compute resolution
        double res = 0.0;
        int n_points = 0;
        int nres;
        std::vector<int> indices2(2);
        std::vector<float> sqr_distances(2);
        pcl::search::KdTree<PointT> tree3;
        tree3.setInputCloud(model);

        for (size_t i = 0; i < model->size(); ++i) {
            if (!pcl_isfinite((*model)[i].x)) {
                continue;
            }
            // Considering the second nearest neighbor since the first is the point itself
            nres = tree3.nearestKSearch(i, 2, indices2, sqr_distances);
            if (nres == 2) {
                res += sqrt(sqr_distances[1]);
                ++n_points;
            }
        }
        if (n_points != 0) {
            res /= n_points;
        }
        // FIXME: make res equal 1
        res = 1;

        // Set up resolution invariance
        if (res != 0.0f) {
            model_ss_ *= res;
            scene_ss_ *= res;
            rf_rad_ *= res;
            descr_rad_ *= res;
            cg_size_ *= res;
        }

        // Compute model normals
        pcl::NormalEstimationOMP<PointT, pcl::Normal> model_norm;
        pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>);
        model_norm.setKSearch(10);
        model_norm.setInputCloud(model);
        model_norm.compute(*model_normals);

        // Downsample to extract keypoints
        pcl::UniformSampling<PointT> uniform_sampling;
        pcl::PointCloud<int>::Ptr model_keyindices(new pcl::PointCloud<int>);
        pcl::PointCloud<PointT>::Ptr model_keypoints(new pcl::PointCloud<PointT>);
        uniform_sampling.setInputCloud(model);
        uniform_sampling.setRadiusSearch(model_ss_);
        uniform_sampling.compute(*model_keyindices);
        pcl::copyPointCloud(*model, model_keyindices->points, *model_keypoints);

        pcl::PointCloud<int>::Ptr scene_keyindices(new pcl::PointCloud<int>);
        pcl::PointCloud<PointT>::Ptr scene_keypoints(new pcl::PointCloud<PointT>);
        uniform_sampling.setInputCloud(cloud_mls);
        uniform_sampling.setRadiusSearch(scene_ss_);
        uniform_sampling.compute(*scene_keyindices);
        pcl::copyPointCloud(*cloud_mls, scene_keyindices->points, *scene_keypoints);

        // Compute descriptors
        pcl::SHOTEstimationOMP<PointT, pcl::Normal, DescriptorType> descr_est;
        pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>);
        pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>);
        descr_est.setRadiusSearch(descr_rad_);
        descr_est.setInputCloud(model);
        descr_est.setInputNormals(model_normals);
        descr_est.setSearchSurface(model);
        descr_est.compute(*model_descriptors);

        descr_est.setInputCloud(cloud_mls);
        descr_est.setInputNormals(mls_normals);
        descr_est.setSearchSurface(cloud_mls);
        descr_est.compute(*scene_descriptors);

        // Find Model-Scene correspondences with KdTree
        pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences);
        pcl::KdTreeFLANN<DescriptorType> match_search;
        match_search.setInputCloud(model_descriptors);

        // For each scene keypoint descriptor, find the nearest neighbor in the model keypoints descriptor cloud and add it to the correspondences vector
        for (size_t i = 0; i < scene_descriptors->size(); ++i) {
            std::vector<int> neigh_indices(1);
            std::vector<float> neigh_sqr_dists(1);
            // skip NaNs
            if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) {
                continue;
            }
            int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
            // Add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
                pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
                model_scene_corrs->push_back(corr);
            }
        }

        // Clustering using Hough 3D Transforms
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;
        // Compute (keypoint) reference frames only for Hough
        pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>);
        pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>);

        pcl::BOARDLocalReferenceFrameEstimation<PointT, pcl::Normal, RFType> rf_est;
        rf_est.setFindHoles(true);
        rf_est.setRadiusSearch(rf_rad_);
        rf_est.setInputCloud(model_keypoints);
        rf_est.setInputNormals(model_normals);
        rf_est.setSearchSurface(model);
        rf_est.compute(*model_rf);
        rf_est.setInputCloud(scene_keypoints);
        rf_est.setInputNormals(mls_normals);
        rf_est.setSearchSurface(cloud_mls);
        rf_est.compute(*scene_rf);
        // // Clustering
        // pcl::Hough3DGrouping<PointT, PointT, RFType, RFType> clusterer;
        // clusterer.setHoughBinSize(cg_size_);
        // clusterer.setHoughThreshold(cg_thresh_);
        // clusterer.setUseInterpolation(true);
        // clusterer.setUseDistanceWeight(false);
        // clusterer.setInputCloud(model_keypoints);
        // clusterer.setInputRF(model_rf);
        // clusterer.setSceneCloud(scene_keypoints);
        // clusterer.setSceneRF(scene_rf);
        // clusterer.setModelSceneCorrespondences(model_scene_corrs);
        // clusterer.recognize(rototranslations, clustered_corrs);

        //==================================
        // Message conversion and publishing
        //==================================
        // DEBUG: view point cloud normals
        // viewer.removePointCloud("normals");
        // viewer.addPointCloudNormals<PointT, pcl::Normal>(model, model_normals, 1, 0.02, "normals");
        viewer.removePointCloud("scene_cloud");
        viewer.removePointCloud("off_scene_model");
        viewer.removePointCloud("scene_keypoints");
        viewer.removePointCloud("off_scene_model_keypoints");
        viewer.addPointCloud(cloud_mls, "scene_cloud");
        pcl::PointCloud<PointT>::Ptr off_scene_model(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointT>);
        // Show keypoints and correspondences
        pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1,0.3,0.5), Eigen::Quaternionf(0.7071068, 0.7071068, 0, 0));
        pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1,0.3,0.5), Eigen::Quaternionf(0.7071068, 0.7071068, 0, 0));
        pcl::visualization::PointCloudColorHandlerCustom<PointT> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
        viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
        // KEYPOINTS
        pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
        viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
        viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");

        viewer.spinOnce(100);

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

    // FIXME: check PCL version
    std::cout << "PCL Version: " << PCL_VERSION_PRETTY << std::endl;

    // Change the PCL verbosity level
    // L_ERROR suppresses the error:
    // [pcl::SHOTEstimation::computeFeature] Theh local reference frame is not valid! Aborting description of point with index XXXX
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    CylinderFinder cylFind;
    ros::spin();

    return 0;
}
