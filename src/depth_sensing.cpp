/*
 * depth_sensing.cpp
 * wheelchair_dacop
 * version: 1.0.0 Majestic Maidenhair
 * Status: Gamma
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include <ros/package.h> //find ROS packages, needs roslib dependency
#include "wheelchair_msgs/mobilenet.h"
#include "wheelchair_msgs/foundObjects.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

//experimental
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

#include <fstream>
#include <iostream>


#include <sstream>
using namespace std;

const int DEBUG_doesWheelchairDumpPkgExist = 0;
const int DEBUG_getResolutionOnStartup = 0;
const int DEBUG_broadcastTransform = 0;
const int DEBUG_getPointDepth = 0;
const int DEBUG_objectDepthCallback = 0;
const int DEBUG_main = 0;

ros::Publisher object_depth_pub;

bool gotResolution = 0; //var flag for successfully getting camera resolution
int imageHeight = 0; //var to store height of rectified image pointcloud
int imageWidth = 0; //var to store width of rectified image pointcloud

struct DetectedObjects {
    string object_name;
    float object_confidence;
    float box_x;
    float box_y;
    float box_width;
    float box_height;
    int totalObjectsInFrame;

    float distance;
    int centerX;
    int centerY;
    int centerZ;

    float pointX;
    float pointY;
    float pointZ;
};

int totalObjectsDetected;

struct DetectedObjects detectedObjects[100];

tf::StampedTransform transform;


//function for printing space sizes
void printSeparator(int spaceSize) {
	if (spaceSize == 0) {
		printf("--------------------------------------------\n");
	}
	else {
		printf("\n");
		printf("--------------------------------------------\n");
		printf("\n");
	}
}

//does the wheelchair dump package exist in the workspace?
void doesWheelchairDumpPkgExist() {
	if (ros::package::getPath("wheelchair_dump") == "") {
		cout << "FATAL:  Couldn't find package 'wheelchair_dump' \n";
		cout << "FATAL:  Closing training_context node. \n";
        if (DEBUG_doesWheelchairDumpPkgExist) {
    		printSeparator(1);
        }
		ros::shutdown();
		exit(0);
	}
}



void getResolutionOnStartup(const sensor_msgs::PointCloud2::ConstPtr& dpth) {
    imageHeight = dpth->height;
    imageWidth = dpth->width;
    if (DEBUG_getResolutionOnStartup) {
        cout << imageHeight << "x" << imageWidth << "\n";
    }
}


void broadcastTransform() {
    wheelchair_msgs::foundObjects fdObj; //wheelchair msg for detected object depth
    //probably send this to the sql node for checking
    int totalObjects = 0;
    for (int isObject = 0; isObject < totalObjectsDetected; isObject++) {
        //float vec_length = sqrt(pow(X,2) + pow(Y,2) + pow(Z,2)); //calculate physical distance from object
        //cout << "vec length is " << vec_length << "\n";

        //check to see if no other object of the same name exists in this space - check bounding box

        //if nothing else exists create a new object id - put into database

        geometry_msgs::Point objectPoint;
        //std::string framename = "target_frame_" + std::to_string(isObject);
        std::string framename = detectedObjects[isObject].object_name + std::to_string(isObject);

        //objectPoint.header.frame_id = framename;
        //objectPoint.header.stamp = ros::Time::now();
        objectPoint.x = detectedObjects[isObject].pointX;
        objectPoint.y = detectedObjects[isObject].pointY;
        objectPoint.z = detectedObjects[isObject].pointZ;

        if (DEBUG_broadcastTransform) {
            cout << "Point is \n" << objectPoint << "\n";
        }

        float r = -1.5708;
        float p = 0;
        float y = -3.1415;

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(objectPoint.x, objectPoint.y, objectPoint.z) );
        tf::Quaternion quat;
        quat.setRPY(r,p,y);  //where r p y are fixed
        transform.setRotation(quat);
        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_depth_link", framename));

        //publish
        fdObj.id.push_back(isObject);
        fdObj.object_name.push_back(detectedObjects[isObject].object_name);
        fdObj.point_x.push_back(detectedObjects[isObject].pointX);
        fdObj.point_y.push_back(detectedObjects[isObject].pointY);
        fdObj.point_z.push_back(detectedObjects[isObject].pointZ);

        fdObj.rotation_r.push_back(r);
        fdObj.rotation_p.push_back(p);
        fdObj.rotation_y.push_back(y);

        if (DEBUG_broadcastTransform) {
            cout << "publish fdObj" << endl;
        }
        totalObjects++;
    }
    fdObj.totalObjects = totalObjects;
    object_depth_pub.publish(fdObj); //publish object depths after frame
}

void getPointDepth(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::mobilenet::ConstPtr& obj) {
    /*  Get depths from bounding box data  */
    tf::StampedTransform tfStamp;
    tf::TransformListener listener;
    int width = dpth->width;
    int height = dpth->height;
    if (DEBUG_getPointDepth) {
        cout << width << " x " << height << "\n";
    }

    for (int isObject = 0; isObject < totalObjectsDetected; isObject++) {
        int centerWidth = detectedObjects[isObject].box_x + detectedObjects[isObject].box_width / 2;
        int centerHeight = detectedObjects[isObject].box_y + detectedObjects[isObject].box_height / 2;
        if (DEBUG_getPointDepth) {
            cout << "pixel to extract is " << centerWidth << " x " << centerHeight << "\n";
        }
        detectedObjects[isObject].centerX = (detectedObjects[isObject].box_x + detectedObjects[isObject].box_width) / 2;
        detectedObjects[isObject].centerY = (detectedObjects[isObject].box_y + detectedObjects[isObject].box_height) / 2;

        float X;
        float Y;
        float Z;

        int arrayPosition = detectedObjects[isObject].centerY*dpth->row_step + detectedObjects[isObject].centerX*dpth->point_step;
        if (DEBUG_getPointDepth) {
            cout << "array position " << arrayPosition << "\n"; //try this out to see if it returns 0 - i.e. top left
        }

        int arrayPosX = arrayPosition + dpth->fields[0].offset; // X has an offset of 0
        int arrayPosY = arrayPosition + dpth->fields[1].offset; // Y has an offset of 4
        int arrayPosZ = arrayPosition + dpth->fields[2].offset; // Z has an offset of 8


        memcpy(&X, &dpth->data[arrayPosX], sizeof(float));
        memcpy(&Y, &dpth->data[arrayPosY], sizeof(float));
        memcpy(&Z, &dpth->data[arrayPosZ], sizeof(float));

        if (DEBUG_getPointDepth) {
            cout << X << " x " << Y << " x " << Z << "\n"; //this is the xyz position of the object
        }
        detectedObjects[isObject].pointX = X;
        detectedObjects[isObject].pointY = Y;
        detectedObjects[isObject].pointZ = Z;
    }
}


void objectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::mobilenet::ConstPtr& obj) {
    //cout << "running time sync \n";
    /*  Get resolution of camera image */
    if (gotResolution == 0) {
        getResolutionOnStartup(dpth);
        gotResolution = 1;
    }

    /*  Deserialise the detected object */
    totalObjectsDetected = obj->totalObjectsInFrame;
    //cout << totalObjectsDetected << "\n";

    for (int isObject = 0; isObject < totalObjectsDetected; isObject++) {
        detectedObjects[isObject].object_name = obj->object_name[isObject];
        detectedObjects[isObject].object_confidence = obj->object_confidence[isObject];
        detectedObjects[isObject].box_x = obj->box_x[isObject];
        detectedObjects[isObject].box_y = obj->box_y[isObject];
        detectedObjects[isObject].box_width = obj->box_width[isObject];
        detectedObjects[isObject].box_height = obj->box_height[isObject];
        if (DEBUG_objectDepthCallback) {
            cout << detectedObjects[isObject].object_name << "\n";
        }
    }

    getPointDepth(dpth, obj);
    broadcastTransform();    
}

int main(int argc, char **argv) {
    //stuff to go here
    ros::init(argc, argv, "object_depth");
    ros::NodeHandle n;

    ros::Rate rate(10.0);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "zed_node/depth/depth_registered", 10);
    //message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "zed_node/point_cloud/cloud_registered", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "wheelchair_robot/point_cloud", 10);
    message_filters::Subscriber<wheelchair_msgs::mobilenet> objects_sub(n, "wheelchair_robot/mobilenet/detected_objects", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, wheelchair_msgs::mobilenet> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> depth_sync(MySyncPolicy(10), depth_sub, objects_sub);
    depth_sync.registerCallback(boost::bind(&objectDepthCallback, _1, _2));
    object_depth_pub = n.advertise<wheelchair_msgs::foundObjects>("wheelchair_robot/object_depth/detected_objects", 1000);

    if (ros::isShuttingDown()) {
        //do something
    }
    if (DEBUG_main) {
        cout << "spin \n";
    }
    ros::spin();
    rate.sleep();

    return 0;
}