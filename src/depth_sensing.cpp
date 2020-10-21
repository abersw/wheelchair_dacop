#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "wheelchair_msgs/mobilenet.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Image.h"

#include <sstream>
using namespace std;

struct DetectedObjects {
    string object_name;
    std_msgs::Float32 object_confidence;
    std_msgs::Float32 box_x;
    std_msgs::Float32 box_y;
    std_msgs::Float32 box_width;
    std_msgs::Float32 box_height;
    std_msgs::Int16 totalObjectsInFrame;
};

struct DetectedObjects detectedObjects[100];

void depthCallback(const sensor_msgs::Image::ConstPtr& dpth) {
    //get a pointer to the depth values casting the data
    //pointer to floating point
    //float* depths = (float*)(&dpth->data[0]);

    //image coordinates of the center pixel
    //int u = dpth->width / 2;
    //int v = dpth->height / 2;

    //linear index of the center pixel
    //int centerIdx = u + dpth->width * v;

    //output the measure
    //ROS_INFO("Center distance : %g m", depths[centerIdx]);

    float* depths = (float*)(&dpth->data[0]);
    cout << "things \n";

}

void objectDepth(const wheelchair_msgs::mobilenet::ConstPtr& obj) {
    int totalObjectsDetected = (int)obj->totalObjectsInFrame;
    int objectNo = 0;
    for (objectNo = 0; objectNo < totalObjectsDetected; objectNo++) {
        //detectedObjects[objectNo].object_name = obj->object_name->[0];
        detectedObjects[objectNo].object_name = obj->object_name[objectNo];
        cout << detectedObjects[objectNo].object_name << "\n";
    }
    //cout << totalObjectsDetected;
    cout << "total objs " << totalObjectsDetected << "\n";
    //cout << "others\n";
}

int main(int argc, char **argv) {
    //stuff to go here
    ros::init(argc, argv, "object_depth");
    ros::NodeHandle n;

    ros::Subscriber subDepth = n.subscribe("/zed_node/depth/depth_registered", 100, depthCallback);
    ros::Subscriber subDetectedObjects = n.subscribe("/wheelchair_robot/mobilenet/detected_objects", 10, objectDepth);
    cout << "spin \n";
    ros::spin();

    return 0;
}