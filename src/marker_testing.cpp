#include "stdio.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <math.h> // for sqrt()
#include <time.h> // for clock_t, clock(), CLOCKS_PER_SEC
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;


class Server
{
private:
    // ROS node
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    ros::Subscriber image_sub;
    ros::Subscriber filename_sub;

    // File saving
    bool store_data;
    clock_t start_time;
    std::string filename;
    std::ofstream datafile;

    // ROS parameters
    std::string image_topic;    // name of topic publishing camera images
    std::string pose_topic;     // name of topic for publishing the pose from the camera data
    double markerLength;        // the length of one side of the marker
                                // the output units of the 'tvec' variable will be the same as marker_length
    std::string camera_name;    // name of the camera
    std::string image_name;     // name of window to display image

    // Marker struct for storing marker parameters
    struct Marker
    {
        int marker_number;
        int aruco_id;
        double size;
    };
    std::vector<Marker> marker_list;

    // Calibration parameters
    cv::Mat K;
    cv::Mat distCoeffs;

public:
    Server() : nh_("~")
    {
        //===== Grab parameters from ROS server =====//
        // Topic for camera image
        nh_.param<double>("marker_length", markerLength, 0.200);
        nh_.param<std::string>("image_name", image_name, "Image");
        nh_.param<std::string>("camera", camera_name, "camera");
        image_topic = camera_name + "/rgb/image_raw";

        // DEBUG: print image topic
        ROS_INFO("Subscribing to image topic: %s", image_topic.c_str());

        // Find markers from searching ROS parameters
        // To add more markers add more marker parameters in numerical order
        // i.e. marker1, marker2, marker3 in a yaml file
        // They need to at least have
        // marker1:
        //     aruco_id: [id number in aruco library]
        //     size: [size in meters]
        int counter = 0;
        bool continue_counting = true;
        while (continue_counting) {
            std::string key;
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
                marker_list.push_back(marker);
            }
            else {
                continue_counting = false;
            }
        }

        for (int i = 0; i < marker_list.size(); ++i) {
            std::cout << "Found marker " << marker_list[i].marker_number << "\n";
            std::cout << "\taruco_id: " << marker_list[i].aruco_id << "\n";
            std::cout << "\tsize: " << marker_list[i].size << "\n";
        }

        //===== Initialize publishers and subscribers =====//
        // Receving image data
        image_sub = nh.subscribe<sensor_msgs::Image>(image_topic.c_str(), 1, &Server::imageCallback, this);
        // Stores data to file and filename is received
        filename_sub = nh.subscribe<std_msgs::String>("/file_name", 1, &Server::save_file_callback, this);

        //===== Load Camera Calibration Data =====//
        // TODO: Update the calibration reading section so that it is able to
        // find all the available cameras. These can be found in cameras.yaml in
        // the setup_tf config folder or in calibration_info.yaml in the
        // 'cameras' package. I'm not sure which one to use yet.
        // Check for calibration parameters in yaml file
        std::string key;
        if (nh.searchParam("/camera1/calibration_rgb/camera_matrix", key)) {
            // Grab all calibration values for intrinsic and distortion matrices
            int rows_k;
            nh.getParam("/camera1/calibration_rgb/camera_matrix/rows", rows_k);
            int cols_k;
            nh.getParam("/camera1/calibration_rgb/camera_matrix/cols", cols_k);
            std::vector<double> K_vec(rows_k*cols_k);
            nh.getParam("/camera1/calibration_rgb/camera_matrix/data", K_vec);
            int rows_d;
            nh.getParam("/camera1/calibration_rgb/distortion_coefficients/rows", rows_d);
            int cols_d;
            nh.getParam("/camera1/calibration_rgb/distortion_coefficients/cols", cols_d);
            std::vector<double> dist_vec(rows_d*cols_d);
            nh.getParam("/camera1/calibration_rgb/distortion_coefficients/data", dist_vec);

            // Convert vectors to OpenCV matrices
            double* K_array = K_vec.data(); // vectors must first be converted to arrays for use in cv::Mat()'s constructor'
            K = cv::Mat(3, 3, CV_64F, K_array).clone();
            double* dist_array = dist_vec.data();
            distCoeffs = cv::Mat(5, 1, CV_64F, dist_array).clone();
        }
        else {
            ROS_INFO("Calibration not found for 'camera1'\nUsing default calibration values.");
            // Default calibration parameters for the Astra camera
            double K_[3][3] =
            { {570.3405151367188, 0.0, 319.5},
            { 0.0,    570.3405151367188,   239.5 },
            { 0.0,    0.0,        1.0 } };
            cv::Mat K = cv::Mat(3, 3, CV_64F, K_).clone();
            double dist_[] = { 0, 0, 0, 0, 0 };
            cv::Mat distCoeffs = cv::Mat(5, 1, CV_64F, dist_).clone();
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg){
        //ROS_INFO("in call back");
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("Could not convert from '%s' to 'picture'" , msg->encoding.c_str());
        }

        // Allocate image.
        cv::Mat image = cv_ptr->image;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
        //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
        //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

        // Run an infinite loop until user hits the ESC key.
        std::vector< int > markerIds;
        std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
        cv::aruco::detectMarkers(
            image, // input image
            dictionary, // type of markers that will be searched for
            markerCorners, // output vector of marker corners
            markerIds, // detected marker IDs
            detectorParams, // algorithm parameters
            rejectedCandidates);
        if (markerIds.size() > 0) {
            // Draw all detected markers.
            cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
            std::vector< cv::Vec3d > rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                markerCorners, // vector of already detected markers corners
                markerLength, // length of the marker's side
                K, // input 3x3 floating-point instrinsic camera matrix K
                distCoeffs, // vector of distortion coefficients of 4, 5, 8 or 12 elements
                rvecs, // array of output rotation vectors
                tvecs); // array of output translation vectors
            // Display pose for the detected marker with id=0.
            for (unsigned int i = 0; i < markerIds.size(); i++) {
                if (markerIds[i] == 3) {
                    //std::cout << "rvec" << i << ": \n";
                    //std::cout << rvecs[i] << std::endl;

                    // rvec to rotation matrix using Rodrigues
                    cv::Mat rotation3x3;
                    cv::Rodrigues(rvecs[i], rotation3x3);
                    // Convert rotation matrix to Euler angles
                    {
                        float sy = sqrt(rotation3x3.at<double>(0,0) * rotation3x3.at<double>(0,0) +  rotation3x3.at<double>(1,0) * rotation3x3.at<double>(1,0) );
                        bool singular = sy < 1e-6; // If
                        float x, y, z;
                        if (!singular)
                        {
                            x = atan2(rotation3x3.at<double>(2,1) , rotation3x3.at<double>(2,2));
                            y = atan2(-rotation3x3.at<double>(2,0), sy);
                            z = atan2(rotation3x3.at<double>(1,0), rotation3x3.at<double>(0,0));
                        }
                        else
                        {
                            x = atan2(-rotation3x3.at<double>(1,2), rotation3x3.at<double>(1,1));
                            y = atan2(-rotation3x3.at<double>(2,0), sy);
                            z = 0;
                        }
                        //std::cout << "rotation: " << Vec3f(x, y, z) << std::endl;
                        //std::cout << "translation: " << tvecs[i] << std::endl;
                        printf("Distance: %0.3f, Y-angle: %0.3f\n", tvecs[i][2], y);

                        //========== Save Data to File ==========//
                        if (store_data) {
                            // Get the current time
                            clock_t now = clock();
                            double current_sec = (double) (now-start_time) / CLOCKS_PER_SEC;
                            std::cout << "Time: " << current_sec << std::endl;
                            if (current_sec < 5.0) {
                                // Store the data to the file
                                //fprintf(datafile, "%0.10f,%0.10f,%0.10f,%0.10f,%0.10f,%0.10f\n", tvecs[i][0], tvecs[i][1], tvecs[i][2], rvecs[i][0], rvecs[i][1], rvecs[i][2]);
                                datafile << std::fixed << std::setprecision(10) << tvecs[i][0] << "," << tvecs[i][1] << "," << tvecs[i][2] << "," << x << "," << y << "," << z << "\n";
                            }
                            else {
                                // Close the file and send a message to the terminal
                                datafile.close();
                                store_data = false;
                                std::cout << "Finished recording data and closed file.\n";
                            }
                        }
                        //========== Save Data to File ==========//
                    }
                    // Convert rotation matrix to quaternion
                    double theta = (double)(sqrt(rvecs[i][0]*rvecs[i][0] + rvecs[i][1]*rvecs[i][1] + rvecs[i][2]*rvecs[i][2]));
                    cv::Vec3d axis;
                    axis[0] = rvecs[i][0]/theta;
                    axis[1] = rvecs[i][1]/theta;
                    axis[2] = rvecs[i][2]/theta;
                    // Calculate quaternion from angle-axis representation
                    double w, x, y, z;
                    w = (0.5)*sqrt((rotation3x3.at<double>(0,0)+rotation3x3.at<double>(1,1)+rotation3x3.at<double>(2,2)) + 1);
                    x = (rotation3x3.at<double>(2,1) - rotation3x3.at<double>(1,2))/(4*w);
                    y = (rotation3x3.at<double>(0,2) - rotation3x3.at<double>(2,0))/(4*w);
                    z = (rotation3x3.at<double>(1,0) - rotation3x3.at<double>(0,1))/(4*w);
                    //std::cout << "magnitude of q: " << sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]) << std::endl;

                    // Broadcast transform of pose for tag
                    static tf::TransformBroadcaster br;
                    tf::Transform transform;
                    transform.setOrigin(tf::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));
                    tf::Quaternion q(x,y,z,w);
                    transform.setRotation(q);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_rgb_optical_frame", "/marker_frame"));
                }

                if (markerIds[i] == 3) {
                    cv::Vec3d r = rvecs[i];
                    cv::Vec3d t = tvecs[i];
                    // Draw coordinate axes.
                    cv::aruco::drawAxis(image,
                        K, distCoeffs, // camera parameters
                        r, t, // marker pose
                        0.5*markerLength); // length of the axes to be drawn
                    // Draw a symbol in the upper right corner of the detected marker.
                    //std::vector<cv::Point3d> pointsInterest, pointsInterest2;
                    //pointsInterest.push_back(cv::Point3d(markerLength / 2, markerLength / 2, 1));
                    //pointsInterest.push_back(cv::Point3d(2.5, 6.0, -4.0));
                    //std::vector<cv::Point2d> p, p2;
                    //cv::projectPoints(pointsInterest, rvecs[i], tvecs[i], K, distCoeffs, p);
                    //cv::projectPoints(pointsInterest2, rvecs[i], tvecs[i], K, distCoeffs, p2);
                    //printf("%f",p[0]);
                    //p[0] = (p[0] + p2[0]) / 2;

                    //cv::drawMarker(image,p[0],cv::Scalar(0, 255, 255),cv::MARKER_STAR, 20,2); // thickness
                    // cv::circle(image,p[0],20,cv::Scalar(0, 255, 255),2);
                }

                if (markerIds[i] == 2) {
                    cv::Vec3d r = rvecs[i];
                    cv::Vec3d t = tvecs[i];
                    // Draw coordinate axes.
                    cv::aruco::drawAxis(image,K, distCoeffs,r, t, 0.5*markerLength); // length of the axes to be drawn
                                         // Draw a symbol in the upper right corner of the detected marker.
                    std::vector<cv::Point3d> pointsInterest, pointsInterest2;
                    //pointsInterest.push_back(cv::Point3d(markerLength / 2, markerLength / 2, 1));
                    pointsInterest.push_back(cv::Point3d(5.5, -1.0, 3));
                    std::vector<cv::Point2d> p, p2;
                    cv::projectPoints(pointsInterest, rvecs[i], tvecs[i], K, distCoeffs, p);
                    //cv::projectPoints(pointsInterest2, rvecs[i], tvecs[i], K, distCoeffs, p2);
                    //printf("%f",p[0]);
                    //p[0] = (p[0] + p2[0]) / 2;

                    cv::drawMarker(image, p[0], cv::Scalar(0, 255, 255),cv::MARKER_STAR, 20,2); // thickness
                    //  cv::circle(image, p[0], 20,cv::Scalar(0, 255, 255), 2);
                }
            }
        }

        cv::imshow("Image", image); // show image
                                    // Wait for x ms (0 means wait until a keypress).
                                    // Returns -1 if no key is hit.

        //imshow("Image",cv_ptr->image);
        cvWaitKey(100);
        //ROS_INFO("in call back");
    }

    void save_file_callback(const std_msgs::StringConstPtr &msg)
    {
        filename = msg->data;
        // Begin storing data
        std::cout << "Starting data collection . . .\n";
        store_data = true;
        std::string pathname = "/home/turtlebot/catkin_ws/src/pose_estimation/src/data/";
        pathname += filename;
        datafile.open(pathname.c_str());
        datafile << "distance x [m],distance y [m],distance z [m],angle x [rad],angle y [rad],angle z [rad]\n";
        start_time = clock();
    }
};




int main(int argc, char* argv[]){

    ros::init(argc,argv,"marker_testing");
    Server server;

    ros::spin();

    return 0;
}
