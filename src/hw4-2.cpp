#include "stdio.h"
#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

//#include <windows.h> // For Sleep()
// Length of one side of a square marker.
const float markerLength = 2.0;

void Image_callback(sensor_msgs::ImageConstPtr  msg){
    ROS_INFO("in call back");
cv_bridge::CvImagePtr cv_ptr;
           try{
              cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
              }
           catch (cv_bridge::Exception& e){
               ROS_ERROR("Could not convert from'%s' to 'picture'" , msg->encoding.c_str());
              }

              printf("This program detects ArUco markers.\n");
              printf("Hit the ESC key to quit.\n");
              //Camera intrinsic matrix (fill in your actual values here).
              //homework intrinsic parameters about focial length and centric coordinates
              double K_[3][3] =
              { {697.2904582057643, 0.0, 312.0656940945775},
              { 0.0,    697.2904582057643,   235.7976140433796 },
              { 0.0,    0.0,        1.0 } };
              cv::Mat K = cv::Mat(3, 3, CV_64F, K_).clone();
              // Distortion coeffs (fill in your actual values here).
              double dist_[] = { 0.07927045640718081, -0.3723751849058924, -0.00166019329654091, -0.007634032119109094, 0 };
              cv::Mat distCoeffs = cv::Mat(5, 1, CV_64F, dist_).clone();
              //cv::VideoCapture cap(0); // open the camera


              // Allocate image.
              cv::Mat image=cv_ptr->image;
              cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
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
                              cv::Vec3d r = rvecs[i];
                              cv::Vec3d t = tvecs[i];
                              // Draw coordinate axes.
                              cv::aruco::drawAxis(image,
                                  K, distCoeffs, // camera parameters
                                  r, t, // marker pose
                                  0.5*markerLength); // length of the axes to be drawn
                                                     // Draw a symbol in the upper right corner of the detected marker.
                              std::vector<cv::Point3d> pointsInterest, pointsInterest2;
                              //pointsInterest.push_back(cv::Point3d(markerLength / 2, markerLength / 2, 1));
                              pointsInterest.push_back(cv::Point3d(2.5, 6.0, -4.0));
                              std::vector<cv::Point2d> p, p2;
                              cv::projectPoints(pointsInterest, rvecs[i], tvecs[i], K, distCoeffs, p);
                              //cv::projectPoints(pointsInterest2, rvecs[i], tvecs[i], K, distCoeffs, p2);
                              //printf("%f",p[0]);
                              //p[0] = (p[0] + p2[0]) / 2;

                             // cv::drawMarker(image,p[0],cv::Scalar(0, 255, 255),cv::MARKER_STAR, 20,2); // thickness
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

                             // cv::drawMarker(image, p[0], cv::Scalar(0, 255, 255),cv::MARKER_STAR, 20,2); // thickness
                            //  cv::circle(image, p[0], 20,cv::Scalar(0, 255, 255), 2);
                          }
                      }
                  }
                  cv::imshow("Image", image); // show image
                                              // Wait for x ms (0 means wait until a keypress).
                                              // Returns -1 if no key is hit.







    //     imshow("Image",cv_ptr->image);
         cvWaitKey(100);
        // ROS_INFO("in call back");

}

int main(int argc, char* argv[]){

ros::init(argc,argv,"picture_markers");

ros::NodeHandle n;
ros::Subscriber image_sub;

//image_sub = n.subscribe("/camera/rgb/image_raw", 5,Image_callback);
image_sub = n.subscribe("/usb_cam/image_raw", 5, Image_callback);





ros::spin();
	return 0;
}
