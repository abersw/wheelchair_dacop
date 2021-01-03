/*
 * cloud_transform.cpp
 * wheelchair_dacop
 * version: 1.0.0 Majestic Maidenhair
 * Status: Gamma
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

tf::TransformListener *listener_ptr;
ros::Publisher pub;


void objectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;
    //STEP 0 Convert sensor_msgs to pcl
    pcl::fromROSMsg(*dpth, cloud_in);
    //STEP 1 Convert xb3 message to center_bumper frame (i think it is better this way)
    tf::StampedTransform tfStamp;
    try {
        listener_ptr->lookupTransform("zed_left_camera_depth_link", dpth->header.frame_id, ros::Time(0), tfStamp);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s\n", ex.what());
    }

    // Transform point cloud
    pcl_ros::transformPointCloud (cloud_in,cloud_trans,tfStamp);
    cloud_trans.header.frame_id="map";
    sensor_msgs::PointCloud2 msg_pub; 
    pcl::toROSMsg(cloud_trans, msg_pub);
    pub.publish(msg_pub);
}

int main(int argc, char** argv) {
    ros::init (argc, argv, "pc2_transformer");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    listener_ptr=&listener;
    ros::Subscriber sub = nh.subscribe("zed_node/point_cloud/cloud_registered", 1, objectDepthCallback);
    pub=nh.advertise<sensor_msgs::PointCloud2>("wheelchair_robot/point_cloud",1000);

    ros::spin();
    return 1;
}