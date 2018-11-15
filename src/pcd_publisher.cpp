/* This code reads data from a '.pcd' file and then publishes it on a ROS topic
 *for viewing in RVIZ or for testing with other nodes.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <vector>

// Set the type of pointcloud here
typedef pcl::PointXYZ PointT;


int main(int argc, char** argv)
{
    // Begin ROS node
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh("~");
    int num_publish = 10;
    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("points", num_publish);

    // Set up PCL objects
    pcl::PCDReader reader; // reads the '.pcd' files
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    // Determine '.pcd' files to be read
    // generate a string vector of file names
    std::vector<std::string> filenames;
    filenames.push_back("/home/csm/Desktop/pcl_test/table_scene_mug_stereo_textured.pcd");

    // Set publishing frequency
    ros::Rate rate = 10; // 30 Hz

    bool repeat = true; // set to true if you want the frames to run continuously
    bool print_end_message = true; // turns false after indicating that the end of the cycle has been reached, keeps from printing this message multiple times
    unsigned long counter = 0; // counts number of times the cycle has repeated
    while (ros::ok()) {
        // Iterate through all files and publish PointClouds
        if (counter < num_publish || repeat) {
            for (int i = 0; i < filenames.size(); ++i) {
                // Read in PointCloud from '.pcd'
                reader.read(filenames.at(i).c_str(), *cloud);

                // Convert to ROS PointCloud2 message
                pcl::PCLPointCloud2 pcl_pc2;
                sensor_msgs::PointCloud2 pointCloudMsg;
                pcl::toPCLPointCloud2(*cloud, pcl_pc2);
                pcl_conversions::fromPCL(pcl_pc2, pointCloudMsg);

                pointCloudMsg.header.seq = i;
                pointCloudMsg.header.stamp = ros::Time::now();
                pointCloudMsg.header.frame_id = "pcd_frame";

                // Publish
                pcd_pub.publish(pointCloudMsg);
            }
            counter++;
        }
        else if (print_end_message) {
            ROS_INFO("Finished publishing pointclouds.");
            print_end_message = false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
