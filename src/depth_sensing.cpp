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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include <tf/LinearMath/Quaternion.h>
#include <cmath>



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


void objectDepthCallback(const sensor_msgs::Image::ConstPtr& dpth, const wheelchair_msgs::mobilenet::ConstPtr& obj) {
    //notes save float pointer of depth to staticesque variable float
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
        cout << detectedObjects[isObject].object_name << "\n";
    }

    /*  Get depths from bounding box data  */
    
    //float* depthsPtr;
    //depthsPtr = (float*)&dpth->data[0];
    for (int isObject = 0; isObject < totalObjectsDetected; isObject++) {
        float* depths = (float*)(&dpth->data[0]);
        //int u = dpth->width / 2;
        //int v = dpth->height / 2;
        int u = (detectedObjects[isObject].box_x + detectedObjects[isObject].box_width) / 2;
        int v = (detectedObjects[isObject].box_y + detectedObjects[isObject].box_height) / 2;
        int centerIdx = u + dpth->width * v;
        ROS_INFO_STREAM("Center distance : " << detectedObjects[isObject].object_name << " is " << depths[centerIdx] << "\n");
        /*
        int centerWidth = detectedObjects[isObject].box_x + detectedObjects[isObject].box_width / 2;
        int centerHeight = detectedObjects[isObject].box_y + detectedObjects[isObject].box_height / 2;
        detectedObjects[isObject].centerX = centerWidth;
        detectedObjects[isObject].centerY = centerHeight;


*
        /*
        Sample pixel layout
            0 1 2
            3 4 5
            6 7 8
        */
        //linear index of the pixel
       // float extractDepths[pixelSampleNo];
        //int centerIdx = 0;
        //centerIdx = centerWidth-1 + imageWidth * centerHeight-1;
        
            //cout << depthsPtr[centerIdx] << "\n";
        //free(depthsPtr);
        //cout << centerIdx << "\n";
        //float myDepth = (float)depthsPtr[centerIdx];
        //float getDepthValue = depthsPtr[centerIdx];
        /*if (!isnan(getDepthValue)) {
            extractDepths[0] =  getDepthValue;//0
        }*/
        
        /*centerIdx = centerWidth + dpth->width * centerHeight-1;
        extractDepths[1] = depthsPtr[centerIdx]; //1
        centerIdx = centerWidth+1 + dpth->width * centerHeight-1;
        extractDepths[2] = depthsPtr[centerIdx]; //2

        centerIdx = centerWidth-1 + dpth->width * centerHeight;
        extractDepths[3] = depthsPtr[centerIdx]; //3
        centerIdx = centerWidth + dpth->width * centerHeight;
        extractDepths[4] = depthsPtr[centerIdx]; //4
        
        centerIdx = centerWidth+1 + dpth->width * centerHeight;
        extractDepths[5] = depthsPtr[centerIdx]; //5

        centerIdx = centerWidth-1 + dpth->width * centerHeight+1;
        extractDepths[6] = depthsPtr[centerIdx]; //6
        centerIdx = centerWidth + dpth->width * centerHeight+1;
        extractDepths[7] = depthsPtr[centerIdx]; //7
        centerIdx = centerWidth+1 + dpth->width * centerHeight+1;
        extractDepths[8] = depthsPtr[centerIdx]; //8*/

        /*int depthsReceived = 0;
        float extractDepth = 0;
        float addAverageDepths = 0;
        for (int isPixel = 0; isPixel < pixelSampleNo; isPixel++) {
            if (isnan(extractDepths[isPixel])) {
                //don't add a nan to the equation
            }
            else {
                addAverageDepths += extractDepths[isPixel];
                depthsReceived++;
            }
        }
        extractDepth = addAverageDepths / depthsReceived;
        detectedObjects[isObject].distance = extractDepth;
        cout << "distance of " << detectedObjects[isObject].object_name << " is " << detectedObjects[isObject].distance << "\n";
*/
        /*  Broadcast transform  */
        //tf::TransformListener listener;
        //tf::TransformBroadcaster br;
/*
        for (int isObject = 0; isObject < totalObjectsDetected; isObject++) {
            tf::StampedTransform tfStamp;
            tf::TransformBroadcaster br;
            /*try{
                listener.lookupTransform("/map", "/base_footprint", ros::Time(0), tfStamp);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }*/
            //https://stackoverflow.com/questions/38909696/2d-coordinate-to-3d-world-coordinate
            //general equation is
            //u = fx * (X / Z) + cx
            //v = fy * (Y / Z) + cy

            //So it is then straightforward to compute the 3D-coordinates:

            //X = Z / fx * (u - cx)
            //Y = Z / fy * (v - cy)
            //[Z = D]
/*
            float r = -1.5708;
            float p = 0;
            float y = -3.1415;

            tfStamp.setOrigin(tf::Vector3(detectedObjects[isObject].centerX, detectedObjects[isObject].centerY, detectedObjects[isObject].distance));
            tf::Quaternion quat;
            quat.setRPY(r,p,y);  //where r p y are fixed 
            tfStamp.setRotation(quat);
            //tfStamp.header.stamp = ros::Time::now();
            string framename = "target_frame "+ isObject;
            if (!isnan(detectedObjects[isObject].distance)) {
                br.sendTransform(tf::StampedTransform(tfStamp, ros::Time::now(), "zed_left_camera_depth_link",framename));
            }
            else {
                //don't publish
            }

            //tfStamp.header.stamp = ros::Time::now();
            //tfStamp.header.frame_id = "zed_left_camera_depth_link";

            //string frameName = "target_frame " + isObject;
            //tfStamp.child_frame_id = frameName;
        }*/
    }
}

int main(int argc, char **argv) {
    //stuff to go here
    ros::init(argc, argv, "object_depth");
    ros::NodeHandle n;

    //ros::Rate rate(20);
    //ros::Subscriber subDepth = n.subscribe("/zed_node/depth/depth_registered", 100, depthCallback);
    //ros::Subscriber subCloud = n.subscribe("/zed_node/point_cloud/cloud_registered", 100, cloudCallback);
    //ros::Subscriber subDetectedObjects = n.subscribe("/wheelchair_robot/mobilenet/detected_objects", 20, objectsFound);
    //ros::Rate rate(20);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "zed_node/depth/depth_registered", 10);
    message_filters::Subscriber<wheelchair_msgs::mobilenet> objects_sub(n, "wheelchair_robot/mobilenet/detected_objects", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, wheelchair_msgs::mobilenet> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> depth_sync(MySyncPolicy(10), depth_sub, objects_sub);
    depth_sync.registerCallback(boost::bind(&objectDepthCallback, _1, _2));

    cout << "spin \n";
    ros::spin();

    return 0;
}