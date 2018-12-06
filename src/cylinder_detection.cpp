/* This code reads in point cloud data, compresses it along the Z axis into the
 * X,Y plane, then performs the circle version of the Hough transform to detect
 * circles from the points.
 *
 * Frames
 ****Draw frames here for reference***
 * image
 * o-->x-------------------------
 * |
 * y
 * |
 * |            x
 * |            |
 * ---------y<--o----------------
 *          camera
 *
 * Note: the image frame has 'x' going to the right and 'y' going down. For the
 * matrices, the rows go down and the columns go right. So the rows represent
 * the 'y' dimension and the cols represent the 'x' dimension.
 */

// TODO: Ways to improve this code
/*
1) Instead of using the opencv image matrix to store where the points are, you could save them in a vector storing the (x,y) index of each point, then just iterate through the vector instead of the entire image. In other words, use a sparse matrix instead of dense. *** See if you can use Sparse Matrices for your images and if that makes it any faster ***
2) Update the code to handle checking out multiple circle locations (by checking the top X maximum points) and then do a sanity-check of some kind to confirm whether it is really the kind of circle we are looking for.
3) Add in the ability to check for circle of different radii. That way you could add a tolerance on the radius or you could search for multiple circle types.
4) Create a method for allowing the user to filter out points except those around the expected area of the roll. You will need to convert camera points into world frame and then do your filtering based on a bounding box
 */

#include <iostream>
#include <string>
#include <math.h>
#include <limits.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


// Define the pointtype used with PointCloud data
typedef pcl::PointXYZ PointT;

// // DEBUG: Visualizers for debugging
// pcl::visualization::PCLVisualizer viewer("Debugging");

class CylinderDetector
{
private:
    //===== ROS Objects
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub; // subscribes to pointcloud data
    ros::Publisher cyl_pub; // publish the position of the cylinder as a pose
    ros::Publisher marker_pub; // publish cylinder marker
    tf::TransformListener tf_listener;
    std::string camera_name;
    std::string default_frame;
    std::string target_frame;

    //===== Tuning Paramters
    float max_distance_x; // m
    float min_distance_x; // m
    float max_distance_y; // m
    float min_distance_y; // m
    float max_distance_z; // m
    float min_distance_z; // m
    float max_fixed_x; // m
    float min_fixed_x; // m
    float max_fixed_y; // m
    float min_fixed_y; // m
    double resolution; // pixels/m, 256 approx. = 1280 pixels / 5 m
    double rotation_resolution; // radians/section
    double circle_radius; // m
    double scale; // scaling factor for the image window
    int num_potentials; // number of potential points to check for selecting if/where a cylinder is present

    //===== Range and Resolution Data
    PointT min_pt; // minimum point in pointcloud
    PointT max_pt; // maximum point in pointcloud
    float x_max;
    float x_min;
    float y_max;
    float y_min;
    float z_max;
    float z_min;
    double x_range; // range for x data
    double y_range; // range for y data
    int x_pixels; // x resolution for image
    int y_pixels; // y resolution for image
    float x_pixel_delta; // length of each pixel in image frame 'x' direction
    float y_pixel_delta; // length of each pixel in image frame 'y' direction
    float y_mirror_min; // y minimum in camera frame mirrored about x axis
    float y_mirror_max; // y maximum in camera frame mirrored about x axis
    int radius_pixels; // number of pixels in the circle radius (based on resolution)
    int accum_x_pixels; // number of pixels in x dimension of accumulator matrix
    int accum_y_pixels; // number of pixels in y dimension of accumulator matrix

    //===== PCL Objects
    pcl::PointCloud<PointT>::Ptr scene_cloud_optical_frame;
    pcl::PointCloud<PointT>::Ptr scene_cloud_unfiltered;
    pcl::PointCloud<PointT>::Ptr scene_cloud;
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    Eigen::Affine3f affine_transform;

    //===== OpenCV Objects
    cv::Mat top_image;
    cv::Mat top_image_fixed;
    cv::Mat top_image_rgb;
    cv::Mat accumulator;
    cv::Mat accumulator_fixed;
    cv::Mat accumulator_rgb;
    std::vector<cv::Point> potentials;

public:
    CylinderDetector() :
    nh_("~"),
    scene_cloud_optical_frame(new pcl::PointCloud<PointT>),
    scene_cloud_unfiltered(new pcl::PointCloud<PointT>),
    scene_cloud(new pcl::PointCloud<PointT>),
    translation(0, 0, 0),
    rotation(1, 0, 0, 0) // w, x, y, z
    {
        // Load Parameters
        nh_.param<std::string>("camera", camera_name, "camera");
        default_frame = camera_name + "_link";
        nh_.param<std::string>("target_frame", target_frame, default_frame);

        // ROS Objects
        std::string point_topic = "/" + camera_name + "/depth/points";
        target_frame.insert(0, "/"); // camera is assumed to be level with the ground, this frame must one where Z is up
        ROS_INFO("Reading depth points from: %s", point_topic.c_str());
        ROS_INFO("Transforming cloud to '%s' frame", target_frame.c_str());
        pc_sub = nh_.subscribe(point_topic.c_str(), 1, &CylinderDetector::pcCallback, this);
        cyl_pub = nh_.advertise<geometry_msgs::PointStamped>("point", 1);
        marker_pub = nh_.advertise<visualization_msgs::Marker>("marker", 1);

        //===== Tuning Paramters =====//
        max_distance_x = 6; // m
        min_distance_x = 0; // m
        max_distance_y = 5; // m
        min_distance_y = -5; // m
        max_distance_z = 10; // m
        min_distance_z = -10; // m
        max_fixed_x = 6.0; // m
        min_fixed_x = 0.5; // m
        max_fixed_y = 2.5; // m
        min_fixed_y = -2.5; // m
        resolution = 256.0; // pixels/m, 256 approx. = 1280 pixels / 5 m
        rotation_resolution = 0.01; // radians/section
        circle_radius = 0.150; // m
        scale = 0.6; // scale the image window
        num_potentials = 5;
        //============================//
    }

    void pcCallback(const sensor_msgs::PointCloud2 &msg)
    {
        //===== Convert PointCloud and Transform Data =====//
        // Convert ROS PointCloud2 message into PCL pointcloud
        rosMsgToPCL(msg, scene_cloud_optical_frame);

        // Transform pointcloud into appropriate frame
        transformPointCloud(scene_cloud_optical_frame, scene_cloud_unfiltered, msg.header.frame_id, target_frame);

        // Get the bounds of the point cloud
        pcl::getMinMax3D(*scene_cloud_unfiltered, min_pt, max_pt);
        x_max = max_pt.x;
        x_min = min_pt.x;
        y_max = max_pt.y;
        y_min = min_pt.y;
        z_max = max_pt.z;
        z_min = min_pt.z;

        // // DEBUG: Print bounds
        // std::cout << "Bounds [min, max]: \n";
        // std::cout << "x: [" << x_min << ", " << x_max << "]\n";
        // std::cout << "y: [" << y_min << ", " << y_max << "]\n";
        // std::cout << "z: [" << z_min << ", " << z_max << "]\n";
        // std::cout << std::endl;
        //=================================================//

        //===== Preprocessing (filter, segment, etc.) =====//
        // Try to remove the ground layer (find the minimum z level and remove a few centimeters up)
        pcl::PassThrough<PointT> pass; // passthrough filter
        pass.setInputCloud(scene_cloud_unfiltered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.100, 0.500);
        pass.filter(*scene_cloud);
        //=================================================//

        //===== Generate 2D Image =====//
        // Determine the grid size based on the desired resolution
        x_range = x_max - x_min;
        y_range = y_max - y_min;

        if (!std::isfinite(x_range)) {
            x_range = 0;
        }
        if (!std::isfinite(y_range)) {
            y_range = 0;
        }

        // Remember that transforming from camera frame to the image frame mirrors the axes, so x pixels depend on the y range and vis-versa
        x_pixels = round(y_range * resolution);
        y_pixels = round(x_range * resolution);
        x_pixel_delta = (y_range / x_pixels); // length of each pixel in image frame 'x' direction
        y_pixel_delta = (x_range / y_pixels); // length of each pixel in image frame 'y' direction

        // Calculate mirrored points for y-values in camera frame (x direction in image frame)
        y_mirror_min = -y_max;
        y_mirror_max = -y_min;

        // Convert points into 2D image and display
        top_image = cv::Mat::zeros(y_pixels, x_pixels, CV_8U); // first index is rows which represent the y dimension of the image

        for (int i = 0; i < scene_cloud->size(); ++i) {
            // Transform camera points to image indices
            int x_index;
            int y_index;
            cameraToImage(scene_cloud->points[i].x, scene_cloud->points[i].y, x_index, y_index);

            // Check that the values are within bounds
            if (x_index >= 0 && x_index <= (x_pixels - 1) && y_index >= 0 && y_index <= (y_pixels - 1)) {
                top_image.at<uint8_t>(cv::Point(x_index, y_index)) = 255; // make sure the type matches with the matrix value type, CV_*U = uint8_t
            }
        }

        //----- Find contours for blob shapes
        // First 'close' the image to fill in thin lines
        cv::Mat structure_element;
        structure_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15,15));
        cv::Mat top_image_close;
        cv::morphologyEx(top_image, top_image_close, cv::MORPH_CLOSE, structure_element);
        std::vector< std::vector<cv::Point> > contours; // stores the vectors of points making up the contours
        std::vector<cv::Vec4i> hierarchy; // vector storing vectors which represent the connection between contours ([i][0] = next, [i][1] = previous, [i][2] = first child, [i][3] = parent)
        findContours(top_image_close, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        cv::Mat top_image_contours = cv::Mat::zeros(top_image_close.size(), CV_8U);
        for (int i = 0; i < contours.size(); ++i) {
            drawContours(top_image_contours, contours, i, cv::Scalar(255, 255, 255), 1, 8, hierarchy, 0);
        }

        //----- Create an image fixed in place based on 'max/min_fixed_x' and 'max/min_fixed_y', for stable visualization
        int x_offset_pixels;
        int y_offset_pixels;
        cv::Mat top_image_fixed = generateFixedImage(top_image, x_offset_pixels, y_offset_pixels);
        //=============================//

        //===== Circle Hough Transform =====//
        // Create an accumulator matrix that adds padding equal to the radius. This will allow the circle center to lie outside the actual image boundaries
        radius_pixels = round(circle_radius*resolution);
        accum_x_pixels = x_pixels + 2*radius_pixels;
        accum_y_pixels = y_pixels + 2*radius_pixels;
        cv::Mat accumulator = cv::Mat::zeros(accum_y_pixels, accum_x_pixels, CV_16U);

        //----- Hough Transform Iteration
        generateAccumulatorUsingImage(top_image_contours, accumulator);
        // generateAccumulatorUsingImage_OldMethod(top_image_contours, accumulator);

        // Scale accumulator matrix for better visualization
        double accum_max;
        double accum_min;
        cv::Point max_loc;
        cv::Point min_loc;
        cv::minMaxLoc(accumulator, &accum_min, &accum_max, &min_loc, &max_loc);
        cv::Mat accumulator_scaled = accumulator*(USHRT_MAX/accum_max);
        accumulator *= (USHRT_MAX/accum_max); // if you change accumulator type you will need to change the max value

        // Get the maximum point of accumulator (center of the circle)
        int circle_index_x;
        int circle_index_y;
        accumulatorToImage(max_loc.x, max_loc.y, circle_index_x, circle_index_y);

        // Generate mask for removing max
        potentials.clear();
        findPointsDeletion(potentials, accumulator);

        // // DEBUG: Hightlight the max point with a circle
        // cv::Mat top_image_rgb;
        // cv::cvtColor(top_image, top_image_rgb, cv::COLOR_GRAY2RGB);
        // cv::circle(top_image_rgb, cv::Point(circle_index_x, circle_index_y), round(circle_radius*resolution), cv::Scalar(0, 0, USHRT_MAX), 1, cv::LINE_8);
        // cv::drawMarker(top_image_rgb, cv::Point(circle_index_x, circle_index_y), cv::Scalar(0, USHRT_MAX, 0), cv::MARKER_CROSS, 10);
        // cv::Mat top_image_fixed_rgb;
        // cv::cvtColor(top_image_fixed, top_image_fixed_rgb, cv::COLOR_GRAY2RGB);
        // cv::circle(top_image_fixed_rgb, cv::Point(circle_index_x + x_offset_pixels, circle_index_y + y_offset_pixels), round(circle_radius*resolution), cv::Scalar(0, 0, USHRT_MAX), 1, cv::LINE_8);
        // cv::drawMarker(top_image_fixed_rgb, cv::Point(circle_index_x + x_offset_pixels, circle_index_y + y_offset_pixels), cv::Scalar(0, USHRT_MAX, 0), cv::MARKER_CROSS, 10);

        // // DEBUG: test visualization
        // viewer.removePointCloud("scene_cloud");
        // viewer.addPointCloud(scene_cloud, "scene_cloud");
        // viewer.addCoordinateSystem(1.0);
        // viewer.spinOnce();

        // // DEBUG: Show image
        // cv::namedWindow("Top Image Fixed", cv::WINDOW_NORMAL);
        // cv::resizeWindow("Top Image Fixed", 700, 700);
        // cv::imshow("Top Image Fixed", top_image_fixed_rgb);
        //
        // cv::namedWindow("Top Image Contours", cv::WINDOW_NORMAL);
        // cv::resizeWindow("Top Image Contours", 700, 700);
        // cv::imshow("Top Image Contours", top_image_contours);
        //
        // cv::namedWindow("Top Image", cv::WINDOW_NORMAL);
        // cv::resizeWindow("Top Image", 700, 700);
        // cv::imshow("Top Image", top_image);
        //
        // cv::namedWindow("Accumulator", cv::WINDOW_NORMAL);
        // cv::resizeWindow("Accumulator", 700, 700);
        // cv::imshow("Accumulator", accumulator_scaled);
        //
        // cvWaitKey(1);
        //==================================//

        //===== Convert Point Back to Camera Frame and Publish =====//
        float camera_frame_x;
        float camera_frame_y;
        imageToCamera(circle_index_x, circle_index_y, camera_frame_x, camera_frame_y);
        geometry_msgs::PointStamped cylinder_point;
        cylinder_point.header = msg.header;
        cylinder_point.header.frame_id = target_frame.c_str();
        cylinder_point.point.x = camera_frame_x;
        cylinder_point.point.y = camera_frame_y;
        cylinder_point.point.z = 0;
        cyl_pub.publish(cylinder_point);

        // DEBUG: Show cylinder marker
        visualization_msgs::Marker cyl_marker;
        cyl_marker.header = cylinder_point.header;
        cyl_marker.id = 1;
        cyl_marker.type = visualization_msgs::Marker::CYLINDER;
        cyl_marker.pose.position = cylinder_point.point;
        cyl_marker.pose.orientation.x = 0;
        cyl_marker.pose.orientation.y = 0;
        cyl_marker.pose.orientation.z = 0;
        cyl_marker.pose.orientation.w = 1.0;
        cyl_marker.scale.x = 0.20;
        cyl_marker.scale.y = 0.20;
        cyl_marker.scale.z = 1.0;
        cyl_marker.color.a = 1.0;
        cyl_marker.color.r = 0.0;
        cyl_marker.color.g = 1.0;
        cyl_marker.color.b = 0.0;
        marker_pub.publish(cyl_marker);
        //==========================================================//
    }

    void rosMsgToPCL(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>::Ptr cloud)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    }

    void transformPointCloud(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>:: Ptr cloud_out, std::string frame_in, std::string frame_out)
    {
        tf::StampedTransform transform;
        try {
            tf_listener.lookupTransform(frame_in.c_str(), frame_out.c_str(), ros::Time(0), transform);
            translation.x() = transform.getOrigin().x();
            translation.y() = transform.getOrigin().y();
            translation.z() = transform.getOrigin().z();
            rotation.w() = transform.getRotation().w();
            rotation.x() = transform.getRotation().x();
            rotation.y() = transform.getRotation().y();
            rotation.z() = transform.getRotation().z();
            rotation = rotation.inverse(); // PCL transforms backwards compared to ROS for some reason, so the rotation must be inverted

            // // DEBUG
            // std::cout << "Obtained transform for '" << pc_frame << "' frame\n";
            // std::cout << "Translation: \n";
            // std::cout << translation << std::endl;
            // std::cout << "Rotation: \n";
            // std::cout << rotation.w() << std::endl;
            // std::cout << rotation.vec() << std::endl;
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        pcl::transformPointCloud(*cloud_in, *cloud_out, translation, rotation);
        cloud_out->header.frame_id = frame_out;
    }

    cv::Mat generateFixedImage(cv::Mat &image, int &x_offset_pixels, int &y_offset_pixels)
    {
        // Find the fixed range, pixels, and offset
        float x_fixed_range = max_fixed_x - min_fixed_x;
        float y_fixed_range = max_fixed_y - min_fixed_y;
        int x_fixed_pixels = round(y_fixed_range * resolution);
        int y_fixed_pixels = round(x_fixed_range * resolution);
        float x_offset = max_fixed_y - y_max;
        float y_offset = max_fixed_x - x_max;
        x_offset_pixels = round(x_offset * resolution);
        y_offset_pixels = round(y_offset * resolution);

        // Determine whether origin and max point for the relative image are within the fixed image bounds or not.
        // For each point:
        // -1 = relative is less than fixed minimum bounds
        //  0 = relative is within fixed bounds
        //  1 = relative is greater than fixed maxmimum bounds
        int rel_origin_x;
        int rel_origin_y;
        int rel_max_x;
        int rel_max_y;
        // Check location of relative image origin x coordinate
        if (x_offset_pixels < 0) { rel_origin_x = -1; }
        else if (x_offset_pixels < x_fixed_pixels) { rel_origin_x = 0; }
        else { rel_origin_x = 1; }
        // Check location of relative image origin y coordinate
        if (y_offset_pixels < 0) { rel_origin_y = -1; }
        else if (y_offset_pixels < y_fixed_pixels) { rel_origin_y = 0; }
        else { rel_origin_y = 1; }
        // Check location of relative image max x coordinate
        if (x_offset_pixels < -x_pixels) { rel_max_x = -1; }
        else if ((x_offset_pixels + x_pixels) < x_fixed_pixels) { rel_max_x = 0; }
        else { rel_max_x = 1; }
        // Check location of relative image max y coordinate
        if (y_offset_pixels < -y_pixels) { rel_max_y = -1; }
        else if ((y_offset_pixels + y_pixels) < y_fixed_pixels) { rel_max_y = 0; }
        else { rel_max_y = 1; }

        // Check if the relative image is completely outside the fixed image bounds (if it's out in one dimension, the whole image will be out)
        int x_lb_rel, x_ub_rel; // starting/ending x indices for relative image
        int x_lb_fixed, x_ub_fixed; // starting/ending x indices for fixed image
        int y_lb_rel, y_ub_rel; // starting/ending y indices for relative image
        int y_lb_fixed, y_ub_fixed; // starting/ending y indices for fixed image
        bool use_bounds = true; // if false, leaves fixed image blank
        if (rel_origin_x == 1 || rel_origin_y == 1 || rel_max_x == -1 || rel_max_y == -1) {
            use_bounds = false;
        }
        else {
            //----- X coordinate conditions
            // Origin is < bounds
            if (rel_origin_x == -1) {
                x_lb_fixed = 0;
                x_lb_rel = -x_offset_pixels;
            }
            // Origin is within bounds
            else {
                x_lb_fixed = x_offset_pixels;
                x_lb_rel = 0;
            }
            // Max is within bounds
            if (rel_max_x == 0) {
                x_ub_fixed = x_pixels + x_offset_pixels;
                x_ub_rel = x_pixels;
            }
            // Max is > bounds
            else {
                x_ub_fixed = x_fixed_pixels;
                x_ub_rel = x_fixed_pixels - x_offset_pixels;
            }
            //----- Y coordinate conditions
            // Origin is < bounds
            if (rel_origin_y == -1) {
                y_lb_fixed = 0;
                y_lb_rel = -y_offset_pixels;
            }
            // Origin is within bounds
            else {
                y_lb_fixed = y_offset_pixels;
                y_lb_rel = 0;
            }
            // Max is within bounds
            if (rel_max_y == 0) {
                y_ub_fixed = y_pixels + y_offset_pixels;
                y_ub_rel = y_pixels;
            }
            // Max is > bounds
            else {
                y_ub_fixed = y_fixed_pixels;
                y_ub_rel = y_fixed_pixels - y_offset_pixels;
            }
        }

        cv::Mat image_fixed = cv::Mat::zeros(y_fixed_pixels, x_fixed_pixels, CV_8U);

        // Copy points from the relative-size image translated into a fixed frame size image
        if (use_bounds) {
            // Assign ranges for copying image points
            cv::Range x_fixed_indices(x_lb_fixed, x_ub_fixed);
            cv::Range x_rel_indices(x_lb_rel, x_ub_rel);
            cv::Range y_fixed_indices(y_lb_fixed, y_ub_fixed);
            cv::Range y_rel_indices(y_lb_rel, y_ub_rel);

            // Create a pointer to the submatrix of interest
            cv::Mat subrange = image_fixed.colRange(x_fixed_indices).rowRange(y_fixed_indices);
            // Copy values into that submatrix
            image(y_rel_indices, x_rel_indices).copyTo(subrange);
        }

        // // DEBUG: Check indices and offsets
        // std::cout << "===== Matrix Info =====\n";
        // std::cout << "Total points seen: " << scene_cloud->points.size() << "\n";
        // std::cout << "Relative Image: (" << top_image.cols << ", " << top_image.rows << ")\n";
        // std::cout << "width: " << x_pixels << "\n";
        // std::cout << "height: " << y_pixels << "\n";
        // std::cout << "\nFixed Image: (" << top_image_fixed.cols << ", " << top_image_fixed.rows << ")\n";
        // std::cout << "width: " << x_fixed_pixels << "\n";
        // std::cout << "height: " << y_fixed_pixels << "\n";
        // std::cout << "=======================\n";
        // std::cout << std::endl;
        // std::cout << "===== Offset Info =====\n";
        // std::cout << "Relative position: \n";
        // std::cout << "(x,y): (" << x_max << ", " << y_max << ")\n";
        // std::cout << "Fixed position: \n";
        // std::cout << "(x,y): (" << max_fixed_x << ", " << max_fixed_y << ")\n";
        // std::cout << "Offset (pixels): (" << x_offset_pixels << ", " << y_offset_pixels << ")\n";
        // std::cout << "=======================\n";
        // std::cout << std::endl;
        // std::cout << "===== Range Info =====\n";
        // std::cout << "State: \n";
        // std::cout << "rel_origin_x: " << rel_origin_x << "\n";
        // std::cout << "rel_origin_y: " << rel_origin_y << "\n";
        // std::cout << "rel_max_x: " << rel_max_x << "\n";
        // std::cout << "rel_max_y: " << rel_max_y << "\n";
        // std::cout << "Relative: \n";
        // std::cout << "x start: " << x_lb_rel << "\n";
        // std::cout << "x end: " << x_ub_rel << "\n";
        // std::cout << "y start: " << y_lb_rel << "\n";
        // std::cout << "y end: " << y_ub_rel << "\n";
        // std::cout << "Fixed: \n";
        // std::cout << "x start: " << x_lb_fixed << "\n";
        // std::cout << "x end: " << x_ub_fixed << "\n";
        // std::cout << "y start: " << y_lb_fixed << "\n";
        // std::cout << "y end: " << y_ub_fixed << "\n";
        // std::cout << "======================\n";
        // std::cout << std::endl;

        return image_fixed;
    }

    void midpointCircleAlgorithm(std::vector<cv::Point2i> &points, int radius, int x_center = 0, int y_center = 0)
    {
        // Performs the Midpoint Circle Algorithm
        // For equation references see:
        // https://www.geeksforgeeks.org/mid-point-circle-drawing-algorithm/
        // https://en.wikipedia.org/wiki/Midpoint_circle_algorithm

        // Center point, used to translate points to shift the circle so it is centered around a desired point
        cv::Point2i center_point(x_center, y_center);

        // Initial point
        int x = radius, y = 0;
        cv::Point2i point(x,y);
        points.push_back(point + center_point);

        // If radius is >0 print mirrored point at four "corners" of the circle
        if (radius > 0) {
            // Going in counter-clockwise order
            point.x = y;
            point.y = x;
            points.push_back(point + center_point);
            point.x = -x;
            point.y = y;
            points.push_back(point + center_point);
            point.x = y;
            point.y = -x;
            points.push_back(point + center_point);
        }

        // Determine whether to decrement x or not
        int radius_error = 1 - radius;
        while (x > y) {
            y++;

            //----- Update the radius error for next point comparison
            // Midpoint is inside or on perimeter
            if (radius_error <= 0) {
                radius_error = radius_error + 2*y + 1;
            }
            // Midpoint is outside of the perimeter
            else {
                x--;
                radius_error = radius_error + 2*y - 2*x + 1;
            }

            // Check if all points have been generated
            if (x < y) {
                break;
            }

            // Generate first set of octant points
            point.x = x;
            point.y = y;
            points.push_back(point + center_point);
            point.x = -x;
            point.y = y;
            points.push_back(point + center_point);
            point.x = x;
            point.y = -y;
            points.push_back(point + center_point);
            point.x = -x;
            point.y = -y;
            points.push_back(point + center_point);

            // If x != y then generate the other half of the octant points. (if they are equal there will be overlap if these points are used)
            if (x != y) {
                point.x = y;
                point.y = x;
                points.push_back(point + center_point);
                point.x = -y;
                point.y = x;
                points.push_back(point + center_point);
                point.x = y;
                point.y = -x;
                points.push_back(point + center_point);
                point.x = -y;
                point.y = -x;
                points.push_back(point + center_point);
            }
        }
    }

    void trigCircleGeneration(std::vector<cv::Point2i> &points, int radius, double resolution, int x_center = 0, int y_center = 0)
    {
        // Center point, used to translate points to shift the circle so it is centered around a desired point
        cv::Point2i center_point(x_center, y_center);

        double theta = 0.0; // current angle around the circle
        while (theta < 2*M_PI) {
            // Calculate the (x,y) position along the circle in the image frame
            int x = round(radius_pixels*cos(theta));
            int y = round(radius_pixels*sin(theta));
            cv::Point2i point(x, y);
            points.push_back(point + center_point);
            theta += resolution;
        }
    }

    void generateAccumulatorUsingImage(cv::Mat &image, cv::Mat &accumulator)
    {
        // Iterate through each point in the image (the image should be of type 'CV_8U', if not change the template type from 'uint8_t' to whatever type matches the matrix image type), for each point that is not 0 create a circle andn increment the pixel values in the accumulator along that circle.
        int num_rows = image.rows;
        int num_cols = image.cols;

        for (int i = 0; i < num_rows; ++i) {
            for (int j = 0; j < num_cols; ++j) {
                if (!(image.at<uint8_t>(i,j) == 0)) {
                    // Convert circle poinst from image index to accumulator index
                    int accum_x;
                    int accum_y;
                    imageToAccumulator(j, i, accum_x, accum_y);

                    // Increment around a circle and store votes
                    std::vector<cv::Point2i> points;
                    //--- Generates points using trigonometric functions and incrementing the angle by user-defined resolution
                    // trigCircleGeneration(points, radius_pixels, rotation_resolution, accum_x, accum_y);
                    //--- Generates points using midpoint circle algorithm, so only the necessary points are generated
                    midpointCircleAlgorithm(points, radius_pixels, accum_x, accum_y);

                    // accumulator.at<uint16_t>(cv::Point(accum_x, accum_y)) += 1; // type must match matrix type CV_16U = uint16_t
                    for (int k = 0; k < points.size(); ++k) {
                        accumulator.at<uint16_t>(points.at(k)) += 1; // type must match matrix type, cv_16U = uint16_t
                    }
                }
            }
        }
    }

    // FIXME: currently left in here to test it's speed against the new methods
    void generateAccumulatorUsingImage_OldMethod(cv::Mat &top_image_accum, cv::Mat accumulator)
    {
        int num_rows = top_image_accum.rows;
        int num_cols = top_image_accum.cols;

        for (int i = 0; i < num_rows; ++i) {
            for (int j = 0; j < num_cols; ++j) {
                if (!(top_image_accum.at<uint8_t>(i,j) == 0)) {
                    // Increment around a circle by rotation_resolution
                    double theta = 0.0; /// current angle around the circle
                    while (theta < 2*M_PI) {
                        // Calculate the (x,y) position along the circle in the image frame
                        int a = j - radius_pixels*cos(theta);
                        int b = i - radius_pixels*sin(theta);
                        // Convert circle poinst from image index to accumulator index
                        int accum_x;
                        int accum_y;
                        imageToAccumulator(a, b, accum_x, accum_y);
                        accumulator.at<uint16_t>(cv::Point(accum_x, accum_y)) += 1; // type must match matrix type CV_16U = uint16_t
                        theta += rotation_resolution;
                    }
                }
            }
        }
    }

    void findPointsDeletion(std::vector<cv::Point> &points, cv::Mat &accumulator)
    {
        //----- Determines the top 'num_potentials' number of points by storing the maximum value in accumulator, setting the points within a circle of the desired radius in the image to 0, then refinding the maximum, deleting around that points, (repeat num_potentials times)
        // The returned 'points' vector is in image frame

        cv::Mat accum_iter = accumulator.clone();
        // cv::Mat accum_mask_debug = cv::Mat(accum_iter.rows, accum_iter.cols, CV_16U, USHRT_MAX);

        for (int i = 0; i < num_potentials; ++i) {
            // Grab Max
            double accum_max;
            double accum_min;
            cv::Point max_loc;
            cv::Point min_loc;
            cv::minMaxLoc(accum_iter, &accum_min, &accum_max, &min_loc, &max_loc);

            // Save point
            int circle_index_x;
            int circle_index_y;
            accumulatorToImage(max_loc.x, max_loc.y, circle_index_x, circle_index_y);
            cv::Point point(circle_index_x, circle_index_y);
            points.push_back(point);

            // Delete accumulator values around previous max
            // Create mask
            cv::Mat accum_mask(accum_iter.rows, accum_iter.cols, CV_16U, USHRT_MAX);

            // Create structuring element as circle
            cv::Size mask_size(2*radius_pixels - 1, 2*radius_pixels - 1);
            cv::Mat circle_mask(mask_size, CV_16U);
            circle_mask = cv::getStructuringElement(cv::MORPH_ELLIPSE, mask_size);
            circle_mask.convertTo(circle_mask, CV_16U);
            circle_mask *= USHRT_MAX;

            bitwise_not(circle_mask, circle_mask);

            // Merge mask of 1's and circle mask
            cv::Rect mask_area(max_loc.x - (radius_pixels - 1), max_loc.y - (radius_pixels - 1), 2*radius_pixels - 1, 2*radius_pixels - 1);
            cv::Mat subrange = accum_mask(mask_area);
            circle_mask.copyTo(subrange);
            // accum_mask(mask_area) = 0;

            // Bitwise AND to remove points inside circle
            bitwise_and(accum_iter, accum_mask, accum_iter);
            // bitwise_and(accum_mask_debug, accum_mask, accum_mask_debug);
        }

        // // DEBUG: show mask
        // cv::namedWindow("Mask", cv::WINDOW_NORMAL);
        // cv::resizeWindow("Mask", 700, 700);
        // cv::imshow("Mask", accum_mask_debug);
        // cvWaitKey(1);
    }

    void findPointsLocalMax()
    {
        // See: https://stackoverflow.com/questions/5550290/find-local-maxima-in-grayscale-image-using-opencv
    }

    void cameraToImage(float camera_x_in, float camera_y_in, int& image_x_out, int& image_y_out)
    {
        // Calculate the x index
        float y_mirror = -camera_y_in;
        float y_translated = y_mirror - y_mirror_min;
        image_x_out = trunc(y_translated/x_pixel_delta);

        // Calculate the y index
        int y_index_flipped = trunc(camera_x_in/y_pixel_delta); // camera frame has positive going up, but image frame has positive going down, so find y with positive up first, then flip it
        image_y_out = (y_pixels - 1) - y_index_flipped;
    }

    void imageToCamera(int image_x_in, int image_y_in, float& camera_x_out, float& camera_y_out)
    {
        // Calculate the x position
        int y_index_flipped = (y_pixels - 1) - image_y_in;
        camera_x_out = (y_index_flipped*y_pixel_delta) + (y_pixel_delta/2); // place at middle of pixel

        // Calculate the y position
        float y_translated = (image_x_in*x_pixel_delta) + (x_pixel_delta/2); // place at middle of pixel
        float y_mirror = y_translated + y_mirror_min;
        camera_y_out = -y_mirror;
    }

    void imageToAccumulator(int image_x_in, int image_y_in, int& accum_x_out, int& accum_y_out)
    {
        // Calculate the x index
        accum_x_out = image_x_in + radius_pixels;

        // Calculate the y index
        accum_y_out = image_y_in + radius_pixels;
    }

    void accumulatorToImage(int accum_x_in, int accum_y_in, int& image_x_out, int& image_y_out)
    {
        // Calculate the x index
        image_x_out = accum_x_in - radius_pixels;

        // Calculate the y index
        image_y_out = accum_y_in - radius_pixels;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cylinder_detection");
    CylinderDetector detector;
    ros::spin();

    return 0;
}
