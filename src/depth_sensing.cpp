#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h" //main ROS library
#include "wheelchair_msgs/mobilenet.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    //get a pointer to the depth values casting the data
    //pointer to floating point
    float* depths = (float*)(&msg->data[0]);

    //image coordinates of the center pixel
    int u = msg->width / 2;
    int v = msg->height / 2;

    //linear index of the center pixel
    int centerIdx = u + msg->width * v;

    //output the measure
    ROS_INFO("Center distance : %g m", depths[centerIdx]);
}

int main(int argc, char **argv) {
    //stuff to go here
    ros::init(argc, argv, "depth_sensing");
    ros::NodeHandle n;

    ros::Subscriber subDepth = n.subscribe("/zed_node/depth/depth_registered", 10, depthCallback);
    ros::Subscriber subDetectedObjects = n.subscribe("/wheelchair_robot/mobilenet/detected_objects")

    ros::spin();

    return 0;
}