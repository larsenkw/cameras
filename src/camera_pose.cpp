#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h> // for sqrt()
#include <numeric> // for accumulate()
#include <vector>
#include <iostream>
#include <string>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

class Server
{
private:
    // ROS structure
    ros::NodeHandle nh;         // for subscribing to public topics outside node namespace
    ros::NodeHandle nh_;        // for publishing topics and gather params in the private namespace
    ros::Subscriber image_sub;
    ros::Publisher camera_pose_pub;
    tf::TransformBroadcaster base_link_broadcaster; // broadcaster to show base_link transform for debugging
    tf::TransformListener listener; // used to obtain necessary static transforms
   // ROS parameters
    std::string image_topic;    // name of topic publishing camera images
    std::string pose_topic;     // name of topic for publishing the pose from the camera data
    double markerLength;        // the length of one side of the marker
                                // the output units of the 'tvec' variable will be the same as marker_length
    std::string camera_name;    // name of the camera
    std::string camera_link;    // name of the camera link,should be [camera_name]_link
    bool debug;                 // TRUE prints the /camera/base_link transform in /odom frame
    bool display_image;         // whether the program should display the camera image in a window
                                // with the marker outline and frame
    std::string image_name;     // name of window to display image

    // Averaging window parameters
    std::vector<int> markers_of_interest; // marker IDs for tags you are using
    std::vector< std::vector<double> > quat_window; // vectors for storing a window of quaternion values for a moving-window average
    int window_size;            // size of the averaging window

    // Calibration parameters


    struct Marker{
        int marker_number;
        int aruco_id;
        double size;
        bool seen;
    };

    std::vector<Marker> marker_list;
    // Calibration parameters
    cv::Mat K;
    cv::Mat distCoeffs;

    tf::Vector3 pose_avg;
    tf::Quaternion rot_avg;

    double sumX;
    double sumY;
    double sumZ;
    double sum_X;
    double sum_Y;
    double sum_Z;
    double sum_W;


public:
    Server() : nh_("~"), quat_window(4), window_size(10)
    {
        //===== Grab parameters from ROS server =====//
        // Topic for camera image
        nh_.param<std::string>("pose_topic", pose_topic, "pose");
        nh_.param<double>("marker_length", markerLength, 0.200);
        nh_.param<bool>("debug", debug, false);
        nh_.param<bool>("display_image", display_image, false);
        nh_.param<std::string>("image_name", image_name, "Image");
        nh_.param<std::string>("camera", camera_name, "camera");
        image_topic = camera_name + "/rgb/image_raw";
        // Append camera name to "_link"
        camera_link = camera_name + "_link";

        ROS_INFO("Subscribing to image topic: %s", image_topic.c_str());


        // FIXME: test topic name
        //std::cout << "image topic name: " << image_topic << std::endl;

        //===== Initialize publishers and subscribers =====//
        // Receving image data
        image_sub = nh.subscribe<sensor_msgs::Image>(image_topic.c_str(), 1, &Server::image_callback, this);
        // Publish base_link pose from camera/marker data
        camera_pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic.c_str(), 1);

        //===== Load Camera Calibration Data =====//
        // TODO: Update the calibration reading section so that it is able to
        // find all the available cameras. These can be found in cameras.yaml in
        // the setup_tf config folder or in calibration_info.yaml in the
        // 'cameras' package. I'm not sure which one to use yet.
        // Check for calibration parameters in yaml file
        std::string key;
        if (nh.searchParam("/" + camera_name + "/calibration_rgb/camera_matrix", key)) {
            // Grab all calibration values for intrinsic and distortion matrices
            int rows_k;
            nh.getParam("/" + camera_name + "/calibration_rgb/camera_matrix/rows", rows_k);
            int cols_k;
            nh.getParam("/" + camera_name + "/calibration_rgb/camera_matrix/cols", cols_k);
            std::vector<double> K_vec(rows_k*cols_k);
            nh.getParam("/" + camera_name + "/calibration_rgb/camera_matrix/data", K_vec);
            int rows_d;
            nh.getParam("/" + camera_name + "/calibration_rgb/distortion_coefficients/rows", rows_d);
            int cols_d;
            nh.getParam("/" + camera_name + "/calibration_rgb/distortion_coefficients/cols", cols_d);
            std::vector<double> dist_vec(rows_d*cols_d);
            nh.getParam("/" + camera_name + "/calibration_rgb/distortion_coefficients/data", dist_vec);

            // Convert vectors to OpenCV matrices
            double* K_array = K_vec.data(); // vectors must first be converted to arrays for use in cv::Mat()'s constructor'
            K = cv::Mat(3, 3, CV_64F, K_array).clone();
            double* dist_array = dist_vec.data();
            distCoeffs = cv::Mat(5, 1, CV_64F, dist_array).clone();
        }
        else {
            ROS_INFO("Calibration not found for '%s'\nUsing default calibration values.", camera_name.c_str());
            // Default calibration parameters for the Astra camera
            double K_[3][3] =
            { {570.3405151367188, 0.0, 319.5},
            { 0.0,    570.3405151367188,   239.5 },
            { 0.0,    0.0,        1.0 } };
            cv::Mat K = cv::Mat(3, 3, CV_64F, K_).clone();
            double dist_[] = { 0, 0, 0, 0, 0 };
            cv::Mat distCoeffs = cv::Mat(5, 1, CV_64F, dist_).clone();
        }

        int counter = 0;
        bool continue_counting = true;


        /*8fstream myfile ("marker_list.yaml");
        std::fstream inf( "marker_list.yaml", std::ios::in );
        while( !inf.eof() ) {
            std::cout << inf.get() << "\n";
        }
        /*inf.close();
        inf.clear();
        inf.open( "ex.txt", std::ios::in );
        char c;
        while( inf >> c ) {
            std::cout << c << "\n";
        }
        while(!eof)*/
        while(continue_counting) {
            std::string key;
            std::stringstream ss;
            ss<<"/marker" << (counter+1);
            std::string marker_name = ss.str();
            if (nh.searchParam(marker_name, key)) {
                counter++;
                int aruco_id;
                nh.getParam(marker_name + "/aruco_id", aruco_id);
                double size;
                nh.getParam(marker_name + "/size", size);
                Marker marker;
                marker.marker_number = counter;
                marker.aruco_id = aruco_id;
                marker.size = size;
                marker.seen = false;
                marker_list.push_back(marker);
            }else {
                continue_counting = false;
            }
        }
        std::cout << counter << "markers found." << std::endl;
        //debuging mesage......
        for(int i =0; i < marker_list.size(); i++) {
            std::cout << "Found marker " << marker_list[i].marker_number << "\n";
            std::cout << "\taruco_id: " << marker_list[i].aruco_id << "\n";
            std::cout << "\tsize: " << marker_list[i].size << "\n";
        }
    }

    void image_callback(const sensor_msgs::ImageConstPtr &msg)
    {


        //===== Convert image message to be usable by OpenCV =====//
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert from '%s' to 'picture'", msg->encoding.c_str());
        }
        // Allocate image
        cv::Mat image = cv_ptr->image;

        //===== Marker detection =====//
        // Select ArUco Tag dictionary
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        // Define variables used in detecting markers
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
        std::vector<int> markerIds;
        std::vector<std::vector <cv::Point2f> > markerCorners, rejectedCandidates;

        //----- Detect Markers -----//
        cv::aruco::detectMarkers(
            image,          // input image
            dictionary,     // type of markers that will be searched for
            markerCorners,  // output vector of marker corners
            markerIds,      // detected marker IDs
            detectorParams, // algorithm parameters
            rejectedCandidates);

        // Cycle through all detected markers and only process the ones that
        // match ids found in marker_list

        //----- Search through detected markers and determine pose -----//
        if (markerIds.size() > 0) {
            // Get position and rotation of marker w.r.t. camera
            std::vector<cv::Vec3d> rvecs, tvecs;
            std::vector<cv::Vec3d> rvecs1(marker_list.size()), tvecs1(marker_list.size());

            // Create vectors for pose data of size of markers_seen
            std::vector<geometry_msgs::Vector3> position(marker_list.size());
            std::vector<geometry_msgs::Vector3> euler(marker_list.size());
            std::vector<geometry_msgs::Quaternion> quat(marker_list.size());
            std::vector<geometry_msgs::PoseWithCovarianceStamped> poses(marker_list.size());

            // Reset marker vector to be all "not seen"
            for (int j = 0; j < marker_list.size(); ++j) {
                marker_list[j].seen = false;
            }

            for(unsigned int i =0; i<markerIds.size(); ++i) {
                for(int j =0; j < marker_list.size(); ++j) {
                    if (markerIds[i] == marker_list[j].aruco_id) {

                        marker_list[j].seen = true;

                        std::vector< std::vector<cv::Point2f> > markerCorners_i;
                        markerCorners_i.push_back(markerCorners[i]);
                        markerLength = marker_list[j].size;


                        cv::aruco::estimatePoseSingleMarkers(
                            markerCorners_i,  // vector of already detected marker corners
                            markerLength,   // length of the marker's side
                            K,              // input 33 floating-point intrinsic camera matrix K
                            distCoeffs,     // vector of distortion coefficients of 4, 5, 8, or 12 elements
                            rvecs,          // array of output rotation vectors (this is essentially angle-axis
                                            // where the x,y,z components, elements 1,2,3, characterize the axis
                                            // while the magnitude of the vector gives the angle)
                            tvecs);
                        // array of output translation vectors
                        rvecs1[j] = rvecs[0];
                        tvecs1[j] = tvecs[0];

                        // Create vectors for pose data of size of markers_seen
                        // int num_markers = relevant_index.size();

                            /*    std::vector<geometry_msgs::Vector3> position(marker_list.size());
                                std::vector<geometry_msgs::Vector3> euler(marker_list.size());
                                std::vector<geometry_msgs::Quaternion> quat(marker_list.size());
                                std::vector<geometry_msgs::PoseWithCovarianceStamped> poses(marker_list.size());*/

                        // Convert rotation vector to euler angles and quaternion
                        //    for (unsigned int i = 0; i < num_mar; i++) {
                        // convert to rotation matrix using Rodrigues function
                        cv::Mat rotation3x3;
                        cv::Rodrigues(rvecs1[j], rotation3x3);

                        // Convert rotation matrix to Euler angles
                        float sy = sqrt(rotation3x3.at<double>(0,0) * rotation3x3.at<double>(0,0) +  rotation3x3.at<double>(1,0) * rotation3x3.at<double>(1,0));
                        bool singular = sy < 1e-6;
                        if (!singular) {
                            euler[j].x = atan2(rotation3x3.at<double>(2,1) , rotation3x3.at<double>(2,2));
                            euler[j].y = atan2(-rotation3x3.at<double>(2,0), sy);
                            euler[j].z = atan2(rotation3x3.at<double>(1,0), rotation3x3.at<double>(0,0));
                        }
                        else {
                            euler[j].x = atan2(-rotation3x3.at<double>(1,2), rotation3x3.at<double>(1,1));
                            euler[j].y = atan2(-rotation3x3.at<double>(2,0), sy);
                            euler[j].z = 0;
                        }

                        // Convert rotation matrix to Quaternion
                        double theta = (double)(sqrt(rvecs1[j][0]*rvecs1[j][0] +
                                                     rvecs1[j][1]*rvecs1[j][1] +
                                                     rvecs1[j][2]*rvecs1[j][2]));

                        cv::Vec3d axis;
                        axis[0] = rvecs1[j][0]/theta;
                        axis[1] = rvecs1[j][1]/theta;
                        axis[2] = rvecs1[j][2]/theta;
                                    // Calculate quaternion from angle-axis representation
                        quat[j].w = (0.5)*sqrt((rotation3x3.at<double>(0,0)+rotation3x3.at<double>(1,1)+rotation3x3.at<double>(2,2)) + 1);
                        quat[j].x = (rotation3x3.at<double>(2,1) - rotation3x3.at<double>(1,2))/(4*quat[j].w);
                        quat[j].y = (rotation3x3.at<double>(0,2) - rotation3x3.at<double>(2,0))/(4*quat[j].w);
                        quat[j].z = (rotation3x3.at<double>(1,0) - rotation3x3.at<double>(0,1))/(4*quat[j].w);

                        // TODO: This code will need to be modified when considering
                        // more than one marker of interest.
                        // Store quaternion in sliding window vector
                        quat_window[0].push_back(quat[j].w);
                        quat_window[1].push_back(quat[j].x);
                        quat_window[2].push_back(quat[j].y);
                        quat_window[3].push_back(quat[j].z);

                                        // Keep window at specified size
                        if (quat_window[0].size() > window_size) {
                            quat_window[0].erase(quat_window[0].begin());
                            quat_window[1].erase(quat_window[1].begin());
                            quat_window[2].erase(quat_window[2].begin());
                            quat_window[3].erase(quat_window[3].begin());
                        }

                        // // FIXME: check window size;
                        // std::cout << "w size: " << quat_window[0].size() << "\n";
                        // std::cout << "x size: " << quat_window[1].size() << "\n";
                        // std::cout << "y size: " << quat_window[2].size() << "\n";
                        // std::cout << "z size: " << quat_window[3].size() << "\n";
                        // std::cout << "--------------------------------" << std::endl;

                                        // Calculate the average of the window
                        geometry_msgs::Quaternion quat_avg;
                        quat_avg.w = std::accumulate(quat_window[0].begin(), quat_window[0].end(), 0.0) / quat_window[0].size();
                        quat_avg.x = std::accumulate(quat_window[1].begin(), quat_window[1].end(), 0.0) / quat_window[1].size();
                        quat_avg.y = std::accumulate(quat_window[2].begin(), quat_window[2].end(), 0.0) / quat_window[2].size();
                        quat_avg.z = std::accumulate(quat_window[3].begin(), quat_window[3].end(), 0.0) / quat_window[3].size();

                        // Generate PoseWithCovarianceStamped message
                        poses[j].pose.pose.position.x = tvecs1[j][0];
                        poses[j].pose.pose.position.y = tvecs1[j][1];
                        poses[j].pose.pose.position.z = tvecs1[j][2];
                        poses[j].pose.pose.orientation.w = quat_avg.w;
                        poses[j].pose.pose.orientation.x = quat_avg.x;
                        poses[j].pose.pose.orientation.y = quat_avg.y;
                        poses[j].pose.pose.orientation.z = quat_avg.z;
                    }
                }
            }

             //----- Calculate the Average pose from all markers -----//
             geometry_msgs::PoseWithCovarianceStamped pose_max;
             geometry_msgs::PoseWithCovarianceStamped pose_min;
             // Using this to define the max and min distance to the camera
             //std::cout<<"this is marker list size : "<< marker_list.size()<<" this is detect size : "<<markerIds.size()<<std::endl;

             std::vector<double> weight_list(marker_list.size());
             double weight_sum;

        for (int j = 0; j < marker_list.size(); j++) {

                if(marker_list[j].seen == true){


                    weight_list[j] = 1 / double(sqrt(poses[j].pose.pose.position.x * poses[j].pose.pose.position.x
                                  + poses[j].pose.pose.position.z * poses[j].pose.pose.position.z
                                  +poses[j].pose.pose.position.y * poses[j].pose.pose.position.y));

                    weight_sum += weight_list[j];
                    //std::cout << "Every marker weight : " << weight_list[j] << std::endl;
                }
        }

            //----- Convert pose of marker to pose of base_link in /odom frame
            // odom_T_base_link = odom_T_marker * (base_link_T_camera * camera_T_marker)^(-1)
            // odom_T_marker = (world_T_odom)^(-1) * world_T_marker
            // world_T_odom: static transform set by .yaml parameters (so it can be saved between shutdowns)
            // world_T_marker: static transform based on marker placement in world
            // base_link_T_camera: static transform based on camera position on robot
            // camera_T_marker: dynamic transform found by this code, (pose_averaged)

            tf::StampedTransform world_T_odom;
            tf::StampedTransform world_T_marker1;
            tf::StampedTransform base_link_T_camera_link;
            tf::StampedTransform camera_link_T_camera_rgb_optical;
            tf::StampedTransform camera_rgb_optical_T_marker1;
            tf::Transform odom_T_base_link;

            // Gather transforms
            try {
                listener.lookupTransform("world", "odom", ros::Time(0), world_T_odom);
                listener.lookupTransform("world", "marker1", ros::Time(0), world_T_marker1);
                listener.lookupTransform("base_link", camera_link, ros::Time(0), base_link_T_camera_link);
                listener.lookupTransform(camera_link, camera_name + "_rgb_optical_frame", ros::Time(0), camera_link_T_camera_rgb_optical);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            tf::Vector3 pos;
            tf::Quaternion rot;

            for (int i =0; i < marker_list.size(); ++i) {
                if(marker_list[i].seen == true){
                    camera_rgb_optical_T_marker1.setOrigin(tf::Vector3(poses[i].pose.pose.position.x,
                                                                       poses[i].pose.pose.position.y,
                                                                       poses[i].pose.pose.position.z));
                    tf::Quaternion q(poses[i].pose.pose.orientation.x,
                                     poses[i].pose.pose.orientation.y,
                                     poses[i].pose.pose.orientation.z,
                                     poses[i].pose.pose.orientation.w);
                    camera_rgb_optical_T_marker1.setRotation(q);

                    // Calculate final transform
                    odom_T_base_link = ((world_T_odom.inverse() * world_T_marker1) * (base_link_T_camera_link * camera_link_T_camera_rgb_optical * camera_rgb_optical_T_marker1).inverse());
                    rot = odom_T_base_link.getRotation();
                    pos = odom_T_base_link.getOrigin();

                    //----- Calculate the Average pose from all markers -----/
                    // Position

                    sumX += pos.getX()*weight_list[i];
                    sumY += pos.getY()*weight_list[i];
                    sumZ += pos.getZ()*weight_list[i];
                    sum_W += rot.w()*weight_list[i];
                      // Orientation x
                    sum_Y += rot.x()*weight_list[i];
                     // Orientation y
                    sum_Y += rot.y()*weight_list[i];
                     // Orientation z
                    sum_Z += rot.z()*weight_list[i];
                }
            }

            geometry_msgs::PoseWithCovarianceStamped pose_base_link;
            pose_base_link.header.frame_id = camera_name + "/base_link";
            pose_base_link.pose.pose.position.x = sumX/weight_sum;//pose_avg.x;
            pose_base_link.pose.pose.position.y = sumY/weight_sum;// pose_avg.y;
            pose_base_link.pose.pose.position.z =  sumZ/weight_sum;//pose_avg.z;
            pose_base_link.pose.pose.orientation.x = sum_X/weight_sum;//rot_avg.x;
            pose_base_link.pose.pose.orientation.y = sum_Y/weight_sum;//rot_avg.y;
            pose_base_link.pose.pose.orientation.z = sum_Z/weight_sum;//rot_avg.z;
            pose_base_link.pose.pose.orientation.w =  sum_W/weight_sum;//rot_avg.w;

            //std::cout<< "pose_base_link: "<<pose_base_link.pose.pose.position.x<<" Y : "<<pose_base_link.pose.pose.position.y<<" Z : "<<pose_base_link.pose.pose.position.z<<std::endl;

            //===== Publish pose =====//
            camera_pose_pub.publish(pose_base_link);

            //----- DEBUG: broadcast the transform here for /odom -> /camera/base_link
            if (debug) {
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(pose_base_link.pose.pose.position.x,
                                                pose_base_link.pose.pose.position.y,
                                                pose_base_link.pose.pose.position.z));
                tf::Quaternion q(pose_base_link.pose.pose.orientation.x,
                                 pose_base_link.pose.pose.orientation.y,
                                 pose_base_link.pose.pose.orientation.z,
                                 pose_base_link.pose.pose.orientation.w);
                transform.setRotation(q);
                base_link_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/" + camera_name + "/base_link"));
            }

            //----- Display image, if parameter is TRUE -----//
            // Draw all detected markers.
            if (display_image) {
                cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
                for (int i = 0; i < marker_list.size(); i++) {
                    if(marker_list[i].seen == true){
                        // Get position and orientation vectors of marker
                        cv::Vec3d r = rvecs1[i];
                        cv::Vec3d t = tvecs1[i];
                        // Draw coordinate axes
                        cv::aruco::drawAxis(image,
                            K, distCoeffs,  // camera parameters
                            r, t,           // marker pose
                            0.5*marker_list[i].size); // length of axes to be drawn
                        // // Draw a symbol in the upper right corner of the marker
                        // std::vector<cv::Point3d> pointsInterest;
                        // pointsInterest.push_back(cv::Point3d(markerLength/2, markerLength/2, 0));
                        // std::vector<cv::Point2d> p;
                        // // Project the point onto the 2D image and store as 'p'
                        // cv::projectPoints(pointsInterest, r, t, K, distCoeffs, p);
                        // cv::drawMarker(image, p[0], cv::Scalar(0, 255, 255), cv::MARKER_STAR, 20, 2);
                        // cv::circle(image, p[0], 20, cv::Scalar(0, 255, 255), 2);
                    }
                }

                // Show the image in a window
                cv::imshow(image_name, image);
                cvWaitKey(100); // wait for x ms (0 means wait until a keypress)
                                // returns -1 if no key is hit;
            }
        }
    }
};





int main(int argc, char* argv[]){

    ros::init(argc,argv,"camera_pose");
    Server server;

    ros::spin();

    return 0;
}
