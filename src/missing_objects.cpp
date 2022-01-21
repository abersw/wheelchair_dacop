/*
 * missing_objects.cpp
 * wheelchair_context
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
 *
*/

#include "tof_tool/tof_tool_box.h"


#include "wheelchair_msgs/objectLocations.h"
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

static const int DEBUG_addObjectLocationsToStruct = 1;
static const int DEBUG_getPointCloudTimestamp = 1;
static const int DEBUG_main = 1;

TofToolBox *tofToolBox;

struct Objects { //struct for publishing topic
    int id; //get object id from ros msg
    string object_name; //get object name/class
    float object_confidence; //get object confidence

    float point_x; //get transform point x
    float point_y; //get transform point y
    float point_z; //get transform point z

    float quat_x; //get transform rotation quaternion x
    float quat_y; //get transform rotation quaternion y
    float quat_z; //get transform rotation quaternion z
    float quat_w; //get transform rotation quaternion w
};
struct Objects objectsFileStruct[100000]; //array for storing object data
int totalObjectsFileStruct = 0; //total objects inside struct

ros::Time camera_timestamp;
double camera_timestamp_sec;

tf::TransformListener *ptrListener; //global pointer for transform listener

void addObjectLocationsToStruct(const wheelchair_msgs::objectLocations::ConstPtr& obLoc) {
    int totalObjectsInMsg = obLoc->totalObjects; //total detected objects in ROS msg
    totalObjectsFileStruct = totalObjectsInMsg; //set message total objects to total objects in file struct
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //iterate through entire msg topic array
        objectsFileStruct[isObject].id = obLoc->id[isObject]; //assign object id to struct
        objectsFileStruct[isObject].object_name = obLoc->object_name[isObject]; //assign object name to struct
        objectsFileStruct[isObject].object_confidence = obLoc->object_confidence[isObject]; //assign object confidence to struct

        objectsFileStruct[isObject].point_x = obLoc->point_x[isObject]; //assign object vector point x to struct
        objectsFileStruct[isObject].point_y = obLoc->point_y[isObject]; //assign object vector point y to struct
        objectsFileStruct[isObject].point_z = obLoc->point_z[isObject]; //assign object vector point z to struct

        objectsFileStruct[isObject].quat_x = obLoc->quat_x[isObject]; //assign object quaternion x to struct
        objectsFileStruct[isObject].quat_y = obLoc->quat_y[isObject]; //assign object quaternion y to struct
        objectsFileStruct[isObject].quat_z = obLoc->quat_z[isObject]; //assign object quaternion z to struct
        objectsFileStruct[isObject].quat_w = obLoc->quat_w[isObject]; //assign object quaternion w to struct

        if (DEBUG_addObjectLocationsToStruct) { //print off debug lines
            cout << "array element in id " << isObject << endl;
            cout << objectsFileStruct[isObject].id << "," <<
                    objectsFileStruct[isObject].object_name << ", " <<
                    objectsFileStruct[isObject].object_confidence << endl;
            cout << objectsFileStruct[isObject].point_x << ", " <<
                    objectsFileStruct[isObject].point_y << ", " <<
                    objectsFileStruct[isObject].point_z << endl;
            cout << objectsFileStruct[isObject].quat_x << ", " <<
                    objectsFileStruct[isObject].quat_y << ", " <<
                    objectsFileStruct[isObject].quat_z << ", " <<
                    objectsFileStruct[isObject].quat_w << endl;
            tofToolBox->printSeparator(0);
        }
    }
}

void getPointCloudTimestamp(const sensor_msgs::PointCloud2::ConstPtr& dpth) {
    camera_timestamp = dpth->header.stamp;
    camera_timestamp_sec = camera_timestamp.toSec();
    if (DEBUG_getPointCloudTimestamp) {
        cout.precision(17);
        cout << "pointcloud timestamp: " << fixed << camera_timestamp << endl;
    }
}

int verifyFielfOfView() {
    tf::StampedTransform translation; //initiate translation for transform object
    //try {
    //    ptrListener->waitForTransform("/map", DETframename, camera_timestamp, ros::Duration(3.0)); //wait a few seconds for ROS to respond
    //    ptrListener->lookupTransform("/map", DETframename, camera_timestamp, translation); //lookup translation of object from map frame
    return 0;
}

/**
 * Callback function triggered by received ROS topic detected objects only
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void detectedObjectsCallback(const wheelchair_msgs::objectLocations obLoc) {
    //
}

/**
 * Callback function triggered by received ROS topic full list of objects
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void objectLocationsCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::objectLocations::ConstPtr& obLoc) {
    //calculate field of view
    //calcualte the orientation of the robot, filter transforms within field of view of the robot
    //iterate through pointcloud and find nearest point to transform, probably in xy coordinate
    //see if transform exists within a range
    getPointCloudTimestamp(dpth);
    addObjectLocationsToStruct(obLoc);
}

int main (int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    ros::init(argc, argv, "missing_objects");
    ros::NodeHandle n;

    ros::Rate rate(10.0);

    //full list of objects
    ros::Subscriber detected_objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 1000, detectedObjectsCallback);
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "/zed/zed_node/point_cloud/cloud_registered", 100); //get transformed pointcloud
    message_filters::Subscriber<wheelchair_msgs::objectLocations> objects_sub(n, "wheelchair_robot/dacop/publish_object_locations/objects", 100); //get mobilenet objects detected
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, wheelchair_msgs::objectLocations> MySyncPolicy; //approximately sync the topic rate
    message_filters::Synchronizer<MySyncPolicy> depth_sync(MySyncPolicy(20), depth_sub, objects_sub); //set sync policy
    depth_sync.registerCallback(boost::bind(&objectLocationsCallback, _1, _2)); //set callback for synced topics

    while(ros::ok()) {
        //tofToolBox->sayHello(); //test function for tof toolbox
        tf::TransformListener listener;
        ptrListener = &listener; //set to global pointer - to access from another function

        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
}
