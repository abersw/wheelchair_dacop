/*
 * transform_pc2map.cpp
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

static const int DEBUG_framename_input = 0;
static const int DEBUG_framename_output = 0;

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
        //transform pointcloud zed_left_camera_frame to map frame
        if (DEBUG_framename_input)
            cout << "input frame id is " << dpth->header.frame_id << endl;
        listener_ptr->lookupTransform("map", dpth->header.frame_id, dpth->header.stamp, tfStamp);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s\n", ex.what());
    }

    // Transform point cloud
    pcl_ros::transformPointCloud (cloud_in,cloud_trans,tfStamp);
    cloud_trans.header.frame_id="map";
    if (DEBUG_framename_output)
        cout << "output frame id is " << cloud_trans.header.frame_id << endl;
    sensor_msgs::PointCloud2 msg_pub;
    msg_pub.header = dpth->header;
    pcl::toROSMsg(cloud_trans, msg_pub);
    pub.publish(msg_pub);
}

int main(int argc, char** argv) {
    ros::init (argc, argv, "pc2_transformer");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    listener_ptr=&listener;
    std::string PARAM_pointcloud_src;
    nh.getParam("/wheelchair_robot/param/pointcloud", PARAM_pointcloud_src);
    ros::Subscriber sub = nh.subscribe(PARAM_pointcloud_src, 100, objectDepthCallback);
    pub=nh.advertise<sensor_msgs::PointCloud2>("wheelchair_robot/point_cloud_map",1000);

    ros::spin();
    return 1;
}
