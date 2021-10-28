/*
 * depth_sensing.cpp
 * wheelchair_dacop
 * version: 1.0.0 Majestic Maidenhair
 * Status: Gamma
*/

#include "tof_tool/tof_tool_box.h"

#include "wheelchair_msgs/mobilenet.h"
#include "wheelchair_msgs/foundObjects.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

#include <math.h>

using namespace std;

const int DEBUG_getResolutionOnStartup = 0;
const int DEBUG_publishObjectLocations = 0;
const int DEBUG_getPointDepth = 0;
const int DEBUG_objectDepthCallback = 0;
const int DEBUG_main = 0;

TofToolBox *tofToolBox;

ros::Publisher object_depth_pub;

bool gotResolution = 0; //var flag for successfully getting camera resolution
int imageHeight = 0; //var to store height of rectified image pointcloud
int imageWidth = 0; //var to store width of rectified image pointcloud

float r = -1.5708; //rotation of object
float p = 0; //pitch of object
float y = -3.1415; //yaw of object

struct DetectedObjects { //struct for containing ros msg from mobilenet node
    string object_name; //mobilenet object name
    float object_confidence; //mobilenet classification confidence
    float box_x; //bounding box sizes
    float box_y;
    float box_width;
    float box_height;
    int totalObjectsInFrame; //total objects inside mobilenet frame

    int centerX; //center of bounding box X position
    int centerY; //center of bounding box Y position

    float pointX; //X position of detected object
    float pointY; //Y position of detected object
    float pointZ; //Z position of detected object
};

int totalObjectsDetected[3]; //0 pre-rosmsg, 1 nan filter post rosmsg, 2 nan filter from pcl

struct DetectedObjects detectedObjects[1000]; //struct array for detected objects


//get resolution of rectified pointcloud image
void getResolutionOnStartup(const sensor_msgs::PointCloud2::ConstPtr& dpth) {
    imageHeight = dpth->height; //get height of pointcloud image
    imageWidth = dpth->width; //get width of pointcloud image
    if (DEBUG_getResolutionOnStartup) {
        cout << imageHeight << "x" << imageWidth << "\n"; //print out height and width if debug flag is true
    }
}


void publishObjectLocations() {
    wheelchair_msgs::foundObjects fdObj; //wheelchair msg for detected object depth
    //probably send this to the sql node for checking
    int totalObjects = 0;
    for (int isObject = 0; isObject < totalObjectsDetected[2]; isObject++) {
        //publish local transform to ros msg for object locations node
        fdObj.id.push_back(isObject);
        fdObj.object_name.push_back(detectedObjects[isObject].object_name);
        fdObj.object_confidence.push_back(detectedObjects[isObject].object_confidence);
        fdObj.point_x.push_back(detectedObjects[isObject].pointX); //assign local transform point x to ros msg
        fdObj.point_y.push_back(detectedObjects[isObject].pointY); //assign local transform point y to ros msg
        fdObj.point_z.push_back(detectedObjects[isObject].pointZ); //assign local transform point z to ros msg

        //assign roll pitch and yaw to ros msg
        fdObj.rotation_r.push_back(r);
        fdObj.rotation_p.push_back(p);
        fdObj.rotation_y.push_back(y);

        if (DEBUG_publishObjectLocations) {
            cout << "publish fdObj" << endl; //print debug line if DEBUG is true
        }
        totalObjects++; //iterate totalobjects
    }
    fdObj.totalObjects = totalObjects; //add total objects to ros msg
    object_depth_pub.publish(fdObj); //publish object depths after frame
}

void getPointDepth(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::mobilenet::ConstPtr& obj) {
    //Get depths from bounding box data
    int objectCounter = 0;
    for (int isObject = 0; isObject < totalObjectsDetected[1]; isObject++) {
        int centerWidth = (int(detectedObjects[isObject].box_x) + int(detectedObjects[isObject].box_width)) / 2; //calculate the centre of the bounding box Y axis (width)
        int centerHeight = (int(detectedObjects[isObject].box_y) + int(detectedObjects[isObject].box_height)) / 2; //calculate the centre of the bounding box X axis (height)
        if (DEBUG_getPointDepth) {
            cout << "pixel to extract is " << centerWidth << " x " << centerHeight << "\n"; //print out height and width centre if DEBUG is true
        }
        detectedObjects[isObject].centerX = centerWidth; //assign centerWidth to struct center X
        detectedObjects[isObject].centerY = centerHeight; //assign centerHeight to struct center Y

        float X = 0;
        float Y = 0;
        float Z = 0;

        int arrayPosition = detectedObjects[isObject].centerY*dpth->row_step + detectedObjects[isObject].centerX*dpth->point_step; //get position of point in rectified image array, corresponding with pointcloud
        if (DEBUG_getPointDepth) {
            cout << "array position " << arrayPosition << "\n"; //try this out to see if it returns 0 - i.e. top left
        }

        int arrayPosX = arrayPosition + dpth->fields[0].offset; // X has an offset of 0
        int arrayPosY = arrayPosition + dpth->fields[1].offset; // Y has an offset of 4
        int arrayPosZ = arrayPosition + dpth->fields[2].offset; // Z has an offset of 8


        memcpy(&X, &dpth->data[arrayPosX], sizeof(float)); //add value from depth point to X
        memcpy(&Y, &dpth->data[arrayPosY], sizeof(float)); //add value from depth point to Y
        memcpy(&Z, &dpth->data[arrayPosZ], sizeof(float)); //add value from depth point to Z

        if (DEBUG_getPointDepth) {
            cout << X << " x " << Y << " x " << Z << "\n"; //this is the xyz position of the object
        }
        /*int arrayPosOffset = 0;
        while ((isnan(X)) && (isnan(Y)) && (isnan(Z))) {
            cout << "NaN detected, set offset to " << arrayPosOffset << endl;
            memcpy(&X, &dpth->data[arrayPosX + arrayPosOffset], sizeof(float)); //add value from depth point to X
            memcpy(&Y, &dpth->data[arrayPosY + arrayPosOffset], sizeof(float)); //add value from depth point to Y
            memcpy(&Z, &dpth->data[arrayPosZ + arrayPosOffset], sizeof(float)); //add value from depth point to Z
            arrayPosOffset += 2;
        }
        arrayPosOffset = 0;*/
        if ((isnan(X)) && (isnan(Y)) && (isnan(Z))) {
            //can't get depth from pointcloud
            //ignore current object - it's faster
        }
        else {
            detectedObjects[isObject].pointX = X; //add pointcloud point position X to struct
            detectedObjects[isObject].pointY = Y; //add pointcloud point position Y to struct
            detectedObjects[isObject].pointZ = Z; //add pointcloud point position Z to struct
            objectCounter++;
        }
    }
    totalObjectsDetected[2] = objectCounter;
}


void objectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::mobilenet::ConstPtr& obj) {
    //cout << "running time sync \n";
    //Get resolution of camera image
    if (gotResolution == 0) {
        getResolutionOnStartup(dpth); //get pointcloud image size
        gotResolution = 1;
    }

    //Deserialise the detected object
    totalObjectsDetected[0] = obj->totalObjectsInFrame; //set totalobjects number from ros msg to global variable
    totalObjectsDetected[1] = 0; //total objects detected after NaN detection
    //cout << totalObjectsDetectedROS << "\n";
    //cout << totalObjectsDetected << endl;

    int objectCounter = 0;
    for (int isObject = 0; isObject < totalObjectsDetected[0]; isObject++) { //iterate through entire ros msg
        if (isnan(obj->box_x[isObject]) &&
            isnan(obj->box_y[isObject]) &&
            isnan(obj->box_width[isObject]) &&
            isnan(obj->box_height[isObject])) { //check to see if any data includes NaN

            cout << "NaN found, skipping object" << endl;
            }
        else {
            detectedObjects[objectCounter].object_name = obj->object_name[isObject]; //add object name to struct
            detectedObjects[objectCounter].object_confidence = obj->object_confidence[isObject]; //add object confidence to struct
            detectedObjects[objectCounter].box_x = obj->box_x[isObject]; //add bounding box x to struct
            detectedObjects[objectCounter].box_y = obj->box_y[isObject]; //add bounding box y to struct
            detectedObjects[objectCounter].box_width = obj->box_width[isObject]; //add bounding box width to struct
            detectedObjects[objectCounter].box_height = obj->box_height[isObject]; //add bounding box height to struct
            objectCounter++;
            if (DEBUG_objectDepthCallback) {
                cout << detectedObjects[objectCounter].object_name << "\n"; //print of object name if DEBUG is true
            }
        }
    }
    totalObjectsDetected[1] = objectCounter;

    getPointDepth(dpth, obj); //pass depth cloud and mobilenet ros msg to get pointcloud information
    publishObjectLocations(); //publish calculated object points
}

int main(int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    ros::init(argc, argv, "object_depth");
    ros::NodeHandle n;

    ros::Rate rate(10.0);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "zed_node/depth/depth_registered", 10);
    //message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "zed_node/point_cloud/cloud_registered", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "/zed/zed_node/point_cloud/cloud_registered", 10); //get transformed pointcloud
    message_filters::Subscriber<wheelchair_msgs::mobilenet> objects_sub(n, "wheelchair_robot/mobilenet/detected_objects", 10); //get mobilenet objects detected
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, wheelchair_msgs::mobilenet> MySyncPolicy; //approximately sync the topic rate
    message_filters::Synchronizer<MySyncPolicy> depth_sync(MySyncPolicy(10), depth_sub, objects_sub); //set sync policy
    depth_sync.registerCallback(boost::bind(&objectDepthCallback, _1, _2)); //set callback for synced topics
    object_depth_pub = n.advertise<wheelchair_msgs::foundObjects>("wheelchair_robot/dacop/depth_sensing/detected_objects", 1000); //publish topic for object locations

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