#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "wheelchair_msgs/mobilenet.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Image.h"

//experimental
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"

#include <sstream>
using namespace std;

struct DetectedObjects {
    string object_name;
    float object_confidence;
    float box_x;
    float box_y;
    float box_width;
    float box_height;
    int totalObjectsInFrame;

    float distance;
};

int totalObjectsDetected;

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

    //image coordinates of the center of the bounding box
    int objectNo = 0;
    for (objectNo = 0; objectNo < totalObjectsDetected; objectNo++) {
        int centerWidth = detectedObjects[objectNo].box_x + detectedObjects[objectNo].box_width / 2;
        int centerHeight = detectedObjects[objectNo].box_y + detectedObjects[objectNo].box_height / 2;

        //linear index of the pixel
        int centerIdx = centerWidth + dpth->width * centerHeight;
        detectedObjects[objectNo].distance = depths[centerIdx];
        cout << "distance of " << detectedObjects[objectNo].object_name << " is " << detectedObjects[objectNo].distance << "\n"; 
    }
    //cout << "dpth" << "\n";
}

void cloudCallback(const geometry_msgs::PointStamped::ConstPtr& cloud) {
    int objectNo = 0;
    for (objectNo = 0; objectNo < totalObjectsDetected; objectNo++) {
        geometry_msgs::PointStamped pt;
        geometry_msgs::PointStamped pt_transformed;

        int centerWidth = detectedObjects[objectNo].box_x + detectedObjects[objectNo].box_width / 2;
        int centerHeight = detectedObjects[objectNo].box_y + detectedObjects[objectNo].box_height / 2;

        pt.header = cloud->header;
        pt.point.x = centerWidth;
        pt.point.y = centerHeight;
        pt.point.z = cloud->point.z;

    }
}

void objectDepth(const wheelchair_msgs::mobilenet::ConstPtr& obj) {
    totalObjectsDetected = (int)obj->totalObjectsInFrame;
    int objectNo = 0;
    for (objectNo = 0; objectNo < totalObjectsDetected; objectNo++) {
        //detectedObjects[objectNo].object_name = obj->object_name->[0];
        detectedObjects[objectNo].object_name = obj->object_name[objectNo];
        detectedObjects[objectNo].object_confidence = obj->object_confidence[objectNo];
        detectedObjects[objectNo].box_x = obj->box_x[objectNo];
        detectedObjects[objectNo].box_y = obj->box_y[objectNo];
        detectedObjects[objectNo].box_width = obj->box_width[objectNo];
        detectedObjects[objectNo].box_height = obj->box_height[objectNo];
        //cout << detectedObjects[objectNo].object_name << "\n";
    }
    //cout << totalObjectsDetected;
    //cout << "total objs " << totalObjectsDetected << "\n";
    //cout << "others\n";
}

int main(int argc, char **argv) {
    //stuff to go here
    ros::init(argc, argv, "object_depth");
    ros::NodeHandle n;

    ros::Subscriber subDepth = n.subscribe("/zed_node/depth/depth_registered", 100, depthCallback);
    //ros::Subscriber subCloud = n.subscribe("/zed_node/point_cloud/cloud_registered", 100, cloudCallback);
    ros::Subscriber subDetectedObjects = n.subscribe("/wheelchair_robot/mobilenet/detected_objects", 10, objectDepth);
    cout << "spin \n";
    ros::spin();

    return 0;
}