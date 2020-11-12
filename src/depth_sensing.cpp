#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "wheelchair_msgs/mobilenet.h"
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

#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include <tf/LinearMath/Quaternion.h>
#include <cmath>
#include <iostream>



#include <sstream>
using namespace std;
//using namespace message_filters;
//using namespace sensor_msgs;

bool gotResolution = 0;
int imageHeight = 0;
int imageWidth = 0;
int pixelSampleNo = 9;

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
};

int totalObjectsDetected;

struct DetectedObjects detectedObjects[100];
tf::StampedTransform transform;

sensor_msgs::PointCloud2 my_pcl;
sensor_msgs::PointCloud2 depth;
pcl::PointCloud < pcl::PointXYZ > pcl_cloud;

pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;

void getResolutionOnStartup(const sensor_msgs::Image::ConstPtr& dpth) {
    imageHeight = dpth->height;
    imageWidth = dpth->width;
    cout << imageHeight << "x" << imageWidth << "\n";
}

void broadcastTransform() {
    for (int isObject = 0; isObject < totalObjectsDetected; isObject++) {
        cout << detectedObjects[isObject].distance << "\n";
        if (!isnan(detectedObjects[isObject].distance)) {
            //tf::TransformListener listener;
            //get transform listener code in here
            
            


            //float x_offset = (detectedObjects[objectId].box_x-(imageWidth/2));
            //float y_offset = (detectedObjects[objectId].box_y-(imageHeight/2));

            //ROS_DEBUG("%.6f, %.6f, %.6f translation", extractDepth, detectedObjects[objectId].box_x, detectedObjects[objectId].box_y);


            /*tfStamp.header.stamp = ros::Time::now();
            tfStamp.header.frame_id = "zed_left_camera_depth_link";

            string frameName = "target_frame " + isObject;
            tfStamp.child_frame_id = frameName;

            tfStamp.transform.translation.x = detectedObjects[isObject].distance;
            tfStamp.transform.translation.y = detectedObjects[isObject].box_x; //was offset x
            tfStamp.transform.translation.z = detectedObjects[isObject].box_y; //was offset y

            tf2::Quaternion quat;
            quat.setRPY(0, 0, 0);
            tfStamp.transform.rotation.x = quat.x();
            tfStamp.transform.rotation.y = quat.y();
            tfStamp.transform.rotation.z = quat.z();
            tfStamp.transform.rotation.w = quat.w();

            br.sendTransform(tfStamp);*/
        }
        else {
            cout << "object returns depth nan - don't broadcast transform \n";
        }
    }



    //float x_offset = (detectedObjects[objectId].box_x-(imageWidth/2));
    //float y_offset = (detectedObjects[objectId].box_y-(imageHeight/2));

    /*ROS_DEBUG("%.6f, %.6f, %.6f translation", extractDepth, detectedObjects[objectId].box_x, detectedObjects[objectId].box_y);

    tfStamp.header.stamp = ros::Time::now();
    tfStamp.header.frame_id = "zed_left_camera_depth_link";

    string frameName = "target_frame";

    tfStamp.child_frame_id = frameName;
    cout << extractDepth << "\n";
    tfStamp.transform.translation.x = extractDepth;
    tfStamp.transform.translation.y = detectedObjects[objectId].box_x;
    tfStamp.transform.translation.z = detectedObjects[objectId].box_y;

    //tf_conversions::transformations quart;
    //quaternion:: quart;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    tfStamp.transform.rotation.x = quat.x();
    tfStamp.transform.rotation.y = quat.y();
    tfStamp.transform.rotation.z = quat.z();
    tfStamp.transform.rotation.w = quat.w();

    br.sendTransform(tfStamp);*/


    /*q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]*/
}


void objectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::mobilenet::ConstPtr& obj) {
    //notes save float pointer of depth to staticesque variable float
    //cout << "running time sync \n";
    /*  Get resolution of camera image */
    if (gotResolution == 0) {
        //getResolutionOnStartup(dpth);
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
        cout << detectedObjects[isObject].object_name << "\n";
    }

    /*  Get depths from bounding box data  */
    my_pcl = *dpth;
    tf::StampedTransform tfStamp;
    tf::TransformListener listener;
    int width = dpth->width;
    int height = dpth->height;
    cout << width << " x " << height << "\n";

    try{
      listener.lookupTransform("/map", "/base_footprint",
                               ros::Time(0), tfStamp);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    for (int isObject = 0; isObject < totalObjectsDetected; isObject++) {
        int centerWidth = detectedObjects[isObject].box_x + detectedObjects[isObject].box_width / 2;
        int centerHeight = detectedObjects[isObject].box_y + detectedObjects[isObject].box_height / 2;
        cout << "pixel to extract is " << centerWidth << " x " << centerHeight << "\n";
        detectedObjects[isObject].centerX = centerWidth;
        detectedObjects[isObject].centerY = centerHeight;

        float X;
        float Y;
        float Z;

        int arrayPosition = detectedObjects[isObject].centerY*dpth->row_step + detectedObjects[isObject].centerX*dpth->point_step;
        cout << "array position " << arrayPosition << "\n"; //try this out to see if it returns 0 - i.e. top left
        
        int arrayPosX = arrayPosition + dpth->fields[0].offset; // X has an offset of 0
        int arrayPosY = arrayPosition + dpth->fields[1].offset; // Y has an offset of 4
        int arrayPosZ = arrayPosition + dpth->fields[2].offset; // Z has an offset of 8


        memcpy(&X, &dpth->data[arrayPosX], sizeof(float));
        memcpy(&Y, &dpth->data[arrayPosY], sizeof(float));
        memcpy(&Z, &dpth->data[arrayPosZ], sizeof(float));

        cout << X << " x " << Y << " x " << Z << "\n";
        float vec_length = sqrt(pow(X,2) + pow(Y,2) + pow(Z,2)); //calculate physical distance from object
        cout << "vec length is " << vec_length << "\n";


        geometry_msgs::Point objectPoint;
        //std::string framename = "target_frame_" + std::to_string(isObject);
        std::string framename = detectedObjects[isObject].object_name;

        //objectPoint.header.frame_id = framename;
        //objectPoint.header.stamp = ros::Time::now();
        objectPoint.x = X;
        objectPoint.y = Y;
        objectPoint.z = Z;

        cout << "Point is \n" << objectPoint << "\n";

        float r = -1.5708;
        float p = 0;
        float y = -3.1415;

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(objectPoint.x, objectPoint.y, objectPoint.z) );
        tf::Quaternion quat;
        quat.setRPY(r,p,y);  //where r p y are fixed
        transform.setRotation(quat);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_frame", framename));

/*
        //tfStamp.setOrigin(tf::Vector3(objectPoint.point.x, objectPoint.point.y, objectPoint.point.z));
        transform.setOrigin(tf::Vector3(X, Y, Z));
        tf::Quaternion quat;
        quat.setRPY(r,p,y);  //where r p y are fixed 
        //tfStamp.setRotation(quat);
        transform.setRotation(quat);

        
        ROS_INFO_STREAM("frame name is " << framename);
        //br.sendTransform(tf::StampedTransform(tfStamp, ros::Time::now(), "map",framename));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "zed_left_camera_depth_link", framename));
        //br.sendTransform(tf::StampedTransform(tfStamp, ros::Time::now(), "map",framename));
        */

    }
    /*tf::Transform transform;
    tf::TransformBroadcaster br;

    for (int i = 0; i < 5; i++) {
        transform.setOrigin( tf::Vector3(i, i, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        string framename = "target_frame_" + std::to_string(i);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", framename));
      }*/
    
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
    cout << "spin \n";
    ros::spin();
    rate.sleep();

    return 0;
}