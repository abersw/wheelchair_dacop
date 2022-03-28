/*
 * object_locations.cpp
 * wheelchair_dacop
 * version: 1.0.0 Majestic Maidenhair
 * Status: Gamma
*/

#include "tof_tool/tof_tool_box.h"

#include "wheelchair_msgs/foundObjects.h"
#include "wheelchair_msgs/objectLocations.h"

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "rviz/validate_floats.h"

#include <math.h>

using namespace std;

//DEBUG LINES - set variable to 1 to enable, 0 to disable
const int DEBUG_objectsListToStruct = 0;
const int DEBUG_forwardCameraTimestamp = 0;
const int DEBUG_translateObjectToMapFrame = 0;
const int DEBUG_print_foundObjects_msg = 0;
const int DEBUG_nan_detector = 1;

const int DEBUG_main = 0;
const int DEBUG_finish_file_printout = 0;

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
struct Objects objectsLocationStruct[1000]; //array for storing object data
int totalObjectsLocationStruct = 0; //total objects inside struct

tf::TransformListener *ptrListener; //global pointer for transform listener

ros::Publisher *ptr_publish_objectLocations; //global pointer for publishing topic

ros::Time camera_timestamp;
double camera_timestamp_sec;

//assign source camera timestamp to global variable for publishing to object locations
void forwardCameraTimestamp(const wheelchair_msgs::foundObjects objects_msg) {
    camera_timestamp = objects_msg.camera_timestamp;
    camera_timestamp_sec = camera_timestamp.toSec();
    if (DEBUG_forwardCameraTimestamp) {
        cout.precision(17);
        cout << "camera timestamp " << fixed << camera_timestamp << endl;
    }
}

/*
 * translateObjectToMapFrame()
 * objects_msg is the entire ROS array msg
 * objectID is the array number for the ROS array msg
 * DETframename is the local detection transform for translating to map frame
*/
void translateObjectToMapFrame(const wheelchair_msgs::foundObjects objects_msg, int objectID, std::string DETframename) {
    std::string msg_object_name;
    msg_object_name = objects_msg.object_name[objectID];
    double msg_object_confidence = objects_msg.object_confidence[objectID];
    tf::StampedTransform translation; //initiate translation for transform object
    try {
        ptrListener->waitForTransform("/map", DETframename, camera_timestamp, ros::Duration(3.0)); //wait a few seconds for ROS to respond
        ptrListener->lookupTransform("/map", DETframename, camera_timestamp, translation); //lookup translation of object from map frame

        int nanFound = tofToolBox->validateTransform(translation);
        if (nanFound) {
            if (DEBUG_nan_detector) {
                cout << "NaN detected whilst converting to map frame" << endl;
            }
        }
        else {
            objectsLocationStruct[totalObjectsLocationStruct].id = totalObjectsLocationStruct;
            objectsLocationStruct[totalObjectsLocationStruct].object_name = msg_object_name;
            objectsLocationStruct[totalObjectsLocationStruct].object_confidence = msg_object_confidence;

            objectsLocationStruct[totalObjectsLocationStruct].point_x = translation.getOrigin().x(); //set translation x to local variable
            objectsLocationStruct[totalObjectsLocationStruct].point_y = translation.getOrigin().y(); //set translation y to local variable
            objectsLocationStruct[totalObjectsLocationStruct].point_z = translation.getOrigin().z(); //set translation z to local variable

            objectsLocationStruct[totalObjectsLocationStruct].quat_x = translation.getRotation().x(); //set rotation x to local variable
            objectsLocationStruct[totalObjectsLocationStruct].quat_y = translation.getRotation().y(); //set rotation y to local variable
            objectsLocationStruct[totalObjectsLocationStruct].quat_z = translation.getRotation().z(); //set rotation z to local variable
            objectsLocationStruct[totalObjectsLocationStruct].quat_w = translation.getRotation().w(); //set rotation w to local variable
            totalObjectsLocationStruct++; //add 1 to total objects in storage struct - ready for next time
        }
    }
    catch (tf::TransformException ex){
        cout << "Couldn't get translation..." << endl; //catchment function if it can't get a translation from the map
        ROS_ERROR("%s",ex.what()); //print error
        ros::Duration(1.0).sleep();
    }
}

std::pair<std::string , int> publishLocalDetectionTransform(const wheelchair_msgs::foundObjects objects_msg, int isObject) {
    //broadcast detected objects in frame
    int nanDetected = 0;
    static tf::TransformBroadcaster br; //initialise broadcaster class
    std::string DETframename = "DET:" + objects_msg.object_name[isObject] + std::to_string(isObject); //add frame DET object name
    tf::Transform localTransform;
    //create local transform from zed camera to object
    //check for NaNs in local DET coordinates
    localTransform.setOrigin( 
            tf::Vector3(objects_msg.point_x[isObject],
                        objects_msg.point_y[isObject],
                        objects_msg.point_z[isObject]) ); //create transform vector
    tf::Quaternion localQuaternion; //initialise quaternion class
    localQuaternion.setRPY(
            objects_msg.rotation_r[isObject],
            objects_msg.rotation_p[isObject],
            objects_msg.rotation_y[isObject]);  //where r p y are fixed
    localTransform.setRotation(localQuaternion); //set quaternion from struct data
    int foundNaN = tofToolBox->validateTransform(localTransform);

    if (foundNaN) {
        //skip object
        nanDetected = 1;
    }
    else {
        br.sendTransform(
                tf::StampedTransform(localTransform,
                                    camera_timestamp,
                                    "zed_camera_center",
                                    DETframename)); //broadcast transform frame from zed camera link
    }
    //end the temporary frame publishing
    return std::make_pair(DETframename, nanDetected);
}

void publishObjectStructMsg() {
    wheelchair_msgs::objectLocations obLoc;
    obLoc.header.stamp = ros::Time::now();
    obLoc.camera_timestamp = camera_timestamp;
    //publish all objects inside struct
    for (int isObject = 0; isObject < totalObjectsLocationStruct; isObject++) { //iterate through entire struct
        obLoc.id.push_back(objectsLocationStruct[isObject].id);
        obLoc.object_name.push_back(objectsLocationStruct[isObject].object_name);
        obLoc.object_confidence.push_back(objectsLocationStruct[isObject].object_confidence);

        obLoc.point_x.push_back(objectsLocationStruct[isObject].point_x);
        obLoc.point_y.push_back(objectsLocationStruct[isObject].point_y);
        obLoc.point_z.push_back(objectsLocationStruct[isObject].point_z);

        obLoc.quat_x.push_back(objectsLocationStruct[isObject].quat_x);
        obLoc.quat_y.push_back(objectsLocationStruct[isObject].quat_y);
        obLoc.quat_z.push_back(objectsLocationStruct[isObject].quat_z);
        obLoc.quat_w.push_back(objectsLocationStruct[isObject].quat_w);
    }
    obLoc.totalObjects = totalObjectsLocationStruct; //set total objects found in struct
    ptr_publish_objectLocations->publish(obLoc); //publish struct as ros msg array
}

//print out entire objects ros msg from depth_sensing node
void printFoundObjectsMsg(const wheelchair_msgs::foundObjects objects_msg, const int isObject) {
    tofToolBox->printSeparator(0);
    cout << "camera_timestamp" << objects_msg.camera_timestamp << endl;
    cout << "ID: " << objects_msg.id[isObject] << endl;
    cout << "Name: " << objects_msg.object_name[isObject] << endl;
    cout << "Confidence: " << objects_msg.object_confidence[isObject] << endl;

    cout << "Point_x: " << objects_msg.point_x[isObject] << endl;
    cout << "Point_y: " << objects_msg.point_y[isObject] << endl;
    cout << "Point_z: " << objects_msg.point_z[isObject] << endl;

    cout << "Rotation_r: " << objects_msg.rotation_r[isObject] << endl;
    cout << "Rotation_p: " << objects_msg.rotation_p[isObject] << endl;
    cout << "Rotation_y: " << objects_msg.rotation_y[isObject] << endl;
}

void objectsDetectedCallback(const wheelchair_msgs::foundObjects objects_msg) {
    //stuff here on each callback
    //if object isn't detected in room - reduce object influence (instead of deleting?)
    forwardCameraTimestamp(objects_msg); //get camera timestamp via message
    int totalObjects = objects_msg.totalObjects; //get quantity of objects in ROS msg
    for (int isObject = 0; isObject < totalObjects; isObject++) { //iterate through entire ROS msg
        if (DEBUG_print_foundObjects_msg) {
            printFoundObjectsMsg(objects_msg, isObject);
        }
        //publish DET transform for detected object
        std::pair< std::string, int> localDetTransform = publishLocalDetectionTransform(objects_msg, isObject);
        std::string DETframename = localDetTransform.first; //get transform name
        int nanDetectedLocalTransform = localDetTransform.second; //return 1 if nan, 0 if normal
        if (!nanDetectedLocalTransform) { //if no nan detected
            translateObjectToMapFrame(objects_msg, isObject, DETframename); //transofrm to map frame
        }
    }
    if (totalObjectsLocationStruct > 0) {
        publishObjectStructMsg(); //publish ROS msg for publish object locations node
    }
    totalObjectsLocationStruct = 0; //set object locations struct back to 0 once translations have been published
}



int main(int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    ros::init(argc, argv, "object_locations");

    ros::NodeHandle n;
    //callback function when objects are detected from depth_sensing
    ros::Subscriber sub = n.subscribe("wheelchair_robot/dacop/depth_sensing/detected_objects", 1000, objectsDetectedCallback);
    //publish to central publishing locations node
    ros::Publisher local_publish_objectLocations = n.advertise<wheelchair_msgs::objectLocations>("wheelchair_robot/dacop/object_locations/detected_objects", 1000);
    //point this local pub variable to global status, so the publish function can access it.
    ptr_publish_objectLocations = &local_publish_objectLocations;

    while (ros::ok()) {
        tf::TransformListener listener;
        ptrListener = &listener; //set to global pointer - to access from another function

        ros::Rate rate(10.0);
        
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spin();
        rate.sleep();
    }
    
    //start closing procedure
    cout << "closing ROS node" << endl;
    if (DEBUG_finish_file_printout) {
        tofToolBox->printSeparator(0);
        cout << "file output" << endl;
        for (int objectNumber = 0; objectNumber < totalObjectsLocationStruct; objectNumber++) {
            cout << objectsLocationStruct[objectNumber].id << "," <<
                    objectsLocationStruct[objectNumber].object_name << "," <<
                    objectsLocationStruct[objectNumber].object_confidence << endl;
            cout << objectsLocationStruct[objectNumber].point_x << ", " <<
                    objectsLocationStruct[objectNumber].point_y << ", " <<
                    objectsLocationStruct[objectNumber].point_z << endl;
            cout << objectsLocationStruct[objectNumber].quat_x << ", " <<
                    objectsLocationStruct[objectNumber].quat_y << ", " <<
                    objectsLocationStruct[objectNumber].quat_z << ", " <<
                    objectsLocationStruct[objectNumber].quat_w << endl;
        }
        tofToolBox->printSeparator(0);
    }
    return 0;
}
