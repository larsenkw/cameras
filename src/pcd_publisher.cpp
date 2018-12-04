/* This code reads data from a '.pcd' file and then publishes it on a ROS topic
 *for viewing in RVIZ or for testing with other nodes.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <vector>
#include <sys/time.h>

// Set the type of pointcloud here
typedef pcl::PointXYZ PointT;

class PCDPublisher
{
private:
    // Parameters
    int num_publish; // number of times to publish (if not looping infinitely)
    std::vector<std::string> filenames; // vector of filename to read and then publish
    std::string save_filename; // name of the file to save to
    bool save_file; // whether to save incoming pointclouds as '.pcd' files
    bool repeat;
    bool print_end_message;
    unsigned long counter;

    // ROS Objects
    ros::NodeHandle nh_;
    ros::Publisher pcd_pub; // publishes pointclouds from '.pcd' files
    ros::Subscriber pc_sub; // subscribes to pointcloud2 messages from ROS for saving to '.pcd' file
    ros::Rate rate; // ROS publishing frequency
    tf::TransformBroadcaster broadcaster;
    tf::Quaternion q;
    tf::Transform transform_optical_lense;
    tf::Transform transform_lense_camera;
    tf::Transform transform_camera_base;

    // PCL Objects
    pcl::PCDReader reader; // reads the '.pcd' files
    pcl::PCDWriter writer; // writes to '.pcd' files
    std::vector<pcl::PointCloud<PointT>::Ptr> clouds;

public:
    PCDPublisher() :
    nh_("~"),
    rate(30) // 30Hz
    {
        // Initialize parameters
        num_publish = 10;
        filenames.push_back("/home/csm/forklift_ws/src/cameras/models/cylinder_test_scene1.pcd");
        // filenames.push_back("/home/csm/forklift_ws/src/cameras/models/cylinder_test_scene2.pcd");
        // filenames.push_back("/home/csm/forklift_ws/src/cameras/models/cylinder_test_scene3.pcd");
        // filenames.push_back("/home/csm/forklift_ws/src/cameras/models/cylinder_test_scene4.pcd");
        // filenames.push_back("/home/csm/forklift_ws/src/cameras/models/cylinder_test_scene5.pcd");

        rate = (1/(rate.expectedCycleTime().toSec()))/filenames.size();

        save_filename = "test.pcd";
        save_file = false;
        repeat = true; // set to true if you want the frames to run continuously
        print_end_message = true; // turns false after indicating that the end of the cycle has been reached, keeps from printing this message multiple times
        counter = 0; // counts number of times the cycle has repeated

        // Initialize ROS objects
        // pcd_pub = nh_.advertise<sensor_msgs::PointCloud2>("points", num_publish);
        pcd_pub = nh_.advertise<sensor_msgs::PointCloud2>("/astrapro/depth/points", num_publish);
        pc_sub = nh_.subscribe("/astrapro/depth/points", 1, &PCDPublisher::pcCallback, this);

        // tf::Quaternion(x, y, z, w)
        // Optical Frame to Lense Frame
        transform_optical_lense.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform_optical_lense.setRotation(tf::Quaternion(-0.5, 0.5, -0.5, 0.5));

        // Lense Frame to Camera Frame
        transform_lense_camera.setOrigin(tf::Vector3(0.0, -0.02, 0.0));
        transform_lense_camera.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

        // Camera Frame to Baselink Frame
        transform_camera_base.setOrigin(tf::Vector3(.0135, 0.01, 0.44));
        transform_camera_base.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

        loadFiles();
    }

    void spin()
    {
        while (ros::ok()) {
            publishClouds();
            publishTransforms();

            ros::spinOnce();
            rate.sleep();
        }
    }

    void loadFiles()
    {
        for (int i = 0; i < filenames.size(); ++i) {
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            reader.read(filenames.at(i).c_str(), *cloud);
            clouds.push_back(cloud);
        }
    }

    void publishClouds()
    {
        // Iterate through all files and publish PointClouds
        if (counter < num_publish || repeat) {
            for (int i = 0; i < clouds.size(); ++i) {
                // Convert to ROS PointCloud2 message
                pcl::PCLPointCloud2 pcl_pc2;
                sensor_msgs::PointCloud2 pointCloudMsg;
                pcl::toPCLPointCloud2(*clouds.at(i), pcl_pc2);
                pcl_conversions::fromPCL(pcl_pc2, pointCloudMsg);

                pointCloudMsg.header.seq = i;
                pointCloudMsg.header.stamp = ros::Time::now();
                // pointCloudMsg.header.frame_id = "pcd_frame";
                pointCloudMsg.header.frame_id = "astrapro_depth_optical_frame";

                // Publish
                // ROS_INFO("Printing frame %d", i);
                pcd_pub.publish(pointCloudMsg);
            }
            counter++;
        }
        else if (print_end_message) {
            ROS_INFO("Finished publishing pointclouds.");
            print_end_message = false;
        }
    }

    void publishTransforms()
    {
        broadcaster.sendTransform(tf::StampedTransform(transform_optical_lense, ros::Time::now(), "astrapro_depth_frame", "astrapro_depth_optical_frame"));
        broadcaster.sendTransform(tf::StampedTransform(transform_lense_camera, ros::Time::now(), "astrapro_link", "astrapro_depth_frame"));
        broadcaster.sendTransform(tf::StampedTransform(transform_camera_base, ros::Time::now(), "base_link", "astrapro_link"));
    }

    void pcCallback(const sensor_msgs::PointCloud2& msg)
    {
        if (save_file) {
            // Convert message to PCL pointcloud
            pcl::PCLPointCloud2 pc2;
            pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);
            pcl_conversions::toPCL(msg, pc2);

            // Save to PCD File
            writer.write(save_filename, pc2);
        }
    }
};

int main(int argc, char** argv)
{
    // Begin ROS node
    ros::init(argc, argv, "pcd_publisher");
    PCDPublisher publisher;
    publisher.spin();

    return 0;
}
