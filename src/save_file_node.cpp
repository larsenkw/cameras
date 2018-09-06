/* This node simply handles receiving the desired name for the .csv file to save
 * the distance and angle data in and signals the 'marker_testing' node to start
 * storing values.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <string>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_file_node");
    ros::NodeHandle nh;
    ros::Publisher pub_filename = nh.advertise<std_msgs::String>("file_name", 1);

    // // Create test file
    // std::ofstream testfile("/home/kyle/catkin_test/src/pose_estimation/src/data/mytestfile.txt");
    // testfile << "My test line\n";
    // std::cout << testfile.is_open();
    // testfile.close();

    while(ros::ok()) {
        std_msgs::String filename;

        // Wait for user to enter file name
        std::cout << "Enter the filename for the next round of data: ";
        std::getline(std::cin, filename.data);

        // Publish name to signal that data from marker_testing should be stored
        pub_filename.publish(filename);

        ros::spinOnce();
    }

    return 0;
}
