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
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>



#include <sstream>
using namespace std;

bool gotResolution = 0;
int imageHeight = 0;
int imageWidth = 0;

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

void getResolutionOnStartup(const sensor_msgs::Image::ConstPtr& dpth) {
    imageHeight = dpth->height;
    imageWidth = dpth->width;
}

void broadcastTransform(int objectId, float extractDepth) {
    geometry_msgs::TransformStamped tfStamp;

    float x_offset = (detectedObjects[objectId].box_x-(imageWidth/2)/1000);
    float y_offset = (detectedObjects[objectId].box_y-(imageHeight/2)/1000);

    ROS_DEBUG("%.6f, %.6f, %.6f translation", extractDepth, x_offset, y_offset);

    tfStamp.header.stamp = ros::Time::now();
    tfStamp.header.frame_id = "zed_left_camera_depth_link";

    string frameName = "target_frame" + objectId;

    tfStamp.child_frame_id = frameName;

    tfStamp.transform.translation.x = extractDepth;
    tfStamp.transform.translation.y = 0-x_offset;
    tfStamp.transform.translation.z = 0-y_offset;

    //tf_conversions::transformations quart;
    //quaternion:: quart;
    tf2::Quaternion quat;
    tfStamp.transform.rotation.x = quat[0];
    tfStamp.transform.rotation.y = quat[1];
    tfStamp.transform.rotation.z = quat[2];
    tfStamp.transform.rotation.w = quat[3];


    /*q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]*/
}

void depthCallback(const sensor_msgs::Image::ConstPtr& dpth) {

    //get image resolution on startup only
    if (gotResolution == 0) {
        getResolutionOnStartup(dpth);
        gotResolution = 1;
    }

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

    //get image height and width

    float* depths = (float*)(&dpth->data[0]);

    //image coordinates of the center of the bounding box
    int objectNo = 0;
    for (objectNo = 0; objectNo < totalObjectsDetected; objectNo++) {
        int centerWidth = detectedObjects[objectNo].box_x + detectedObjects[objectNo].box_width / 2;
        int centerHeight = detectedObjects[objectNo].box_y + detectedObjects[objectNo].box_height / 2;

        //linear index of the pixel
        int centerIdx = centerWidth + dpth->width * centerHeight;
        float extractDepth = depths[centerIdx];
        //check if pixel is nan
        if (isnan(extractDepth)) {
            //what should I do if NaN is detected - probably take an average from several pixels?
        }
        else {
            detectedObjects[objectNo].distance = extractDepth;
            cout << "distance of " << detectedObjects[objectNo].object_name << " is " << detectedObjects[objectNo].distance << "\n";
            broadcastTransform(objectNo, extractDepth);
        }
        
    }
    //cout << "dpth" << "\n";
}

/*void cloudCallback(const geometry_msgs::PointStamped::ConstPtr& cloud) {
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
}*/

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

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster tb;

    ros::Subscriber subDepth = n.subscribe("/zed_node/depth/depth_registered", 100, depthCallback);
    //ros::Subscriber subCloud = n.subscribe("/zed_node/point_cloud/cloud_registered", 100, cloudCallback);
    ros::Subscriber subDetectedObjects = n.subscribe("/wheelchair_robot/mobilenet/detected_objects", 10, objectDepth);
    cout << "spin \n";
    ros::spin();

    return 0;
}