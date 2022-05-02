/*
 * missing_objects.cpp
 * wheelchair_dacop
 * version: 1.0.0 Majestic Maidenhair
 * Status: Alpha
 *
*/

#include "tof_tool/tof_tool_box.h"


#include "wheelchair_msgs/objectLocations.h"
#include "wheelchair_msgs/missingObjects.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/cache.h>

#include "tf/transform_listener.h"

#include <thread>
#include <ros/callback_queue.h>

#include <math.h>

using namespace std;

static const int DEBUG_getResolutionOnStartup = 0;
static const int DEBUG_rosPrintSequence = 0;
static const int DEBUG_addObjectLocationsToStruct = 0;
static const int DEBUG_addObjectLocationsToStruct_objects = 0;
static const int DEBUG_getPointCloudTimestamp = 0;
static const int DEBUG_getCameraTranslation = 0;
static const int DEBUG_detectedObjectsCallback = 0;
static const int DEBUG_findMatchingPoints = 0;
static const int DEBUG_findMatchingPoints_rawValues = 0;
static const int DEBUG_findMatchingPoints_detectedPoints = 0;
static const int DEBUG_getCorrespondingObjectFrame_cache = 1;
static const int DEBUG_getCorrespondingObjectFrame_redetectedObjects = 0;
static const int DEBUG_transformsFoundInPointcloudDistance = 0;
static const int DEBUG_transformsFoundInPointcloudDistance_detections = 0;
static const int DEBUG_printAllObjects = 0;
static const int DEBUG_printRedetectedObjects = 0;
static const int DEBUG_printMissingObjects = 0;
static const int DEBUG_main = 0;

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


struct FOV {
    double minDepth = 0.1;
    double maxDepth = 5;
    //https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-
    //HD-720p
    int fovx = 85; //horizontal field of view in deg
    int fovy = 54; //vertical field of view in deg

    int gotResolution = 0;
    int imageHeight;
    int imageWidth;
    long numberOfPixels;
};
struct FOV fov;

struct Boundary {
    double visualMaxBoundaryX = 2.0; //objects must be within this distance from the camera - X axis
    double visualMaxBoundaryY = 2.0; //objects must be within this distance from the camera - Y axis

    double pointBoundaryX = 0.1;
    double pointBoundaryY = 0.1;

    double timeRangeReverseValue = 0.1;
    double timeRangeForwardValue = 0.5;
};
struct Boundary boundary;

struct TransformPoints {
    int id;
    std::string object_name;
    int64_t totalCorrespondingPoints = 0;

    int foundFlag = 0;
};

struct MatchingPoints {
    struct TransformPoints objectsList[1000]; //all objects to be added to list for filtering
    int totalObjectsList = 0; //total objects found in pointcloud
    struct TransformPoints objectsRedetected[1000]; //filtered list of objects classified as redetected
    int totalObjectsRedetected = 0; //total of objects redetected
    struct TransformPoints objectsNotRedetected[1000]; //filtered list of objects classified as not redetected
    int totalObjectsNotRedetected = 0; //total of objects not redetected
};
struct MatchingPoints matchingPoints;

tf::TransformListener *ptrListener; //global pointer for transform listener

//https://docs.ros.org/en/api/message_filters/html/c++/classmessage__filters_1_1Cache.html
message_filters::Cache<wheelchair_msgs::objectLocations> cache; //buffer incoming detected objects

boost::shared_ptr<ros::AsyncSpinner> detected_objects_spinner;
boost::shared_ptr<ros::AsyncSpinner> listed_objects_spinner;

ros::Publisher *ptr_pub_objectsList;
ros::Publisher *ptr_pub_objectsRedetected;
ros::Publisher *ptr_pub_objectsNotRedetected;

tf::StampedTransform cameraTranslation;

//get resolution of rectified pointcloud image
void getResolutionOnStartup(const sensor_msgs::PointCloud2::ConstPtr& dpth) {
    if (!fov.gotResolution) {
        fov.imageHeight = dpth->height; //get height of pointcloud image
        fov.imageWidth = dpth->width; //get width of pointcloud image
        fov.numberOfPixels = fov.imageHeight * fov.imageWidth;
        if (DEBUG_getResolutionOnStartup) {
            cout << fov.imageHeight << "x" << fov.imageWidth << "\n"; //print out height and width if debug flag is true
        }
        fov.gotResolution = 1;
    }
}

void rosPrintSequence(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::objectLocations::ConstPtr &obLoc) {
    tofToolBox->printSeparator(0);
    cout << "depth header seq:  " << dpth->header.seq << endl;
    cout << "object header seq: " << obLoc->header.seq << endl;
    tofToolBox->printSeparator(0);
}

void addObjectLocationsToStruct(const wheelchair_msgs::objectLocations::ConstPtr& obLoc) {
    int totalObjectsInMsg = obLoc->totalObjects; //total detected objects in ROS msg
    totalObjectsFileStruct = totalObjectsInMsg; //set message total objects to total objects in file struct
    if (DEBUG_addObjectLocationsToStruct) {
        cout << "total objects in msg are " << totalObjectsFileStruct << endl;
    }
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

        if (DEBUG_addObjectLocationsToStruct_objects) { //print off debug lines
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

/**
 * Function resets matching points total variables back to 0
 */
void resetMatchingPoints() {
    //reset found flags
    for (int isObject = 0; isObject < matchingPoints.totalObjectsList; isObject++) {
        matchingPoints.objectsList[isObject].foundFlag = 0;
        matchingPoints.objectsList[isObject].totalCorrespondingPoints = 0;
    }
    for (int isObject = 0; isObject < matchingPoints.totalObjectsRedetected; isObject++) {
        matchingPoints.objectsRedetected[isObject].foundFlag = 0;
        matchingPoints.objectsRedetected[isObject].totalCorrespondingPoints = 0;
    }
    for (int isObject = 0; isObject < matchingPoints.totalObjectsNotRedetected; isObject++) {
        matchingPoints.objectsNotRedetected[isObject].foundFlag = 0;
        matchingPoints.objectsNotRedetected[isObject].totalCorrespondingPoints = 0;
    }
    matchingPoints.totalObjectsList = 0;
    matchingPoints.totalObjectsRedetected = 0;
    matchingPoints.totalObjectsNotRedetected = 0;
}

void getCameraTranslation() {
    tf::TransformListener listener;
    try{
        listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0)); //wait a few seconds for ROS to respond
        listener.lookupTransform("map", "base_link", ros::Time(0), cameraTranslation); //lookup translation of object from map frame
        if (DEBUG_getCameraTranslation) {
            cout << "camera translation: " << cameraTranslation.getOrigin().x() << ", " << cameraTranslation.getOrigin().y() << endl;
        }
    }
    catch (tf::TransformException ex){
            cout << "Couldn't get camera translation..." << endl; //catchment function if it can't get a translation from the map
            ROS_ERROR("%s",ex.what()); //print error
            ros::Duration(1.0).sleep();
        }
}

/**
 * Callback function triggered by received ROS topic detected objects only
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void detectedObjectsCallback(const wheelchair_msgs::objectLocations::ConstPtr &obLoc) {
    cache.add(obLoc);
    if (DEBUG_detectedObjectsCallback) {
        std::cout << "Oldest time cached is " << cache.getOldestTime() << std::endl;
        std::cout << "Last time received is " << cache.getLatestTime() << std::endl << std::endl;
    }
}

/**
 * Function checks visual object detection to see if object id and transform exist
 * within same 'frame'
 *
 * @param parameter 'isObject' is the position of the object current being
 * iterated through findMatchingPoints
 */
void getCorrespondingObjectFrame(int isObject) {
    ros::Duration timeRangeReverse(boundary.timeRangeReverseValue);
    ros::Duration timeRangeForward(boundary.timeRangeForwardValue);
    ros::Time reverseTime(camera_timestamp - timeRangeReverse); //create boundary back in time
    ros::Time forwardTime(camera_timestamp + timeRangeForward); //create boundary forward in time

    //check to see if cache will not return a null
    if (cache.getElemAfterTime(reverseTime) != NULL) {
        //get ros msg after the specified time 'reverseTime'
        const wheelchair_msgs::objectLocations::ConstPtr &obLoc = cache.getElemAfterTime(reverseTime);
        //double check to see msg header stamp isn't too far away from specified time
        if ((obLoc->header.stamp > reverseTime) && (obLoc->header.stamp < forwardTime)) {
            int totalDet = obLoc->totalObjects; //get total number of objects detected
            for (int isDetObject = 0; isDetObject < totalDet; isDetObject++) {
                int detObjectID = obLoc->id[isDetObject]; //get object ID detected
                //only need to query object ID, object publisher node deals with 3D bounding boxes
                //if object detected and pc2 point close to transform is equal
                if (detObjectID == objectsFileStruct[isObject].id) {
                    if (DEBUG_getCorrespondingObjectFrame_redetectedObjects) {
                        cout <<
                        objectsFileStruct[isObject].id << ":" <<
                        objectsFileStruct[isObject].object_name <<
                        " HAS BEEN REDETECTED!" << endl;
                    }


                    if (matchingPoints.totalObjectsRedetected == 0) {
                        //add first element to array
                        //assign object id to local array
                        matchingPoints.objectsRedetected[matchingPoints.totalObjectsRedetected].id = objectsFileStruct[isObject].id;
                        matchingPoints.objectsRedetected[matchingPoints.totalObjectsRedetected].object_name = objectsFileStruct[isObject].object_name;
                        matchingPoints.objectsRedetected[matchingPoints.totalObjectsRedetected].totalCorrespondingPoints++;
                        matchingPoints.totalObjectsRedetected++; //iterate to next element in array

                    }
                    else {
                        //run through for loop to see if id has already been detected
                        int objectAlreadyInStruct = 0;
                        for (int localDet = 0; localDet < matchingPoints.totalObjectsRedetected; localDet++) {
                            if (objectsFileStruct[isObject].id == matchingPoints.objectsRedetected[localDet].id) {
                                //id is already in struct
                                objectAlreadyInStruct = 1;
                                //add 1 to total points with corresponding transform
                                matchingPoints.objectsRedetected[localDet].totalCorrespondingPoints++;
                            }
                        }
                        if (objectAlreadyInStruct == 0) {
                            //object needs adding to objects redetected struct
                            matchingPoints.objectsRedetected[matchingPoints.totalObjectsRedetected].id = objectsFileStruct[isObject].id;
                            matchingPoints.objectsRedetected[matchingPoints.totalObjectsRedetected].object_name = objectsFileStruct[isObject].object_name;
                            matchingPoints.objectsRedetected[matchingPoints.totalObjectsRedetected].totalCorrespondingPoints++;
                            matchingPoints.totalObjectsRedetected++; //iterate to next element in array
                        }
                        else if (objectAlreadyInStruct == 1) {
                            //don't do anything, object already in objects redetected struct
                        }
                    }
                }
            }
        }
    }
    else {
        //couldn't get ROS detected objects msg from cache
        if (DEBUG_getCorrespondingObjectFrame_cache) {
            cout << "couldn't get detected objects from cache" << endl;
        }
    }
}

/**
 * Function adds all transforms with corresponding pc2 points to array
 *
 * @param parameter 'isObject' is the position of the object current being
 * iterated through findMatchingPoints
 */
void transformsFoundInPointcloud(int isObject) {
    if (matchingPoints.totalObjectsList == 0) { //set object id to first element in array
        matchingPoints.objectsList[matchingPoints.totalObjectsList].id = objectsFileStruct[isObject].id;
        matchingPoints.objectsList[matchingPoints.totalObjectsList].object_name = objectsFileStruct[isObject].object_name;
        matchingPoints.objectsList[matchingPoints.totalObjectsList].totalCorrespondingPoints++;
        matchingPoints.totalObjectsList++; //iterate to next item in array
    }
    else {
        int objectAlreadyInList = 0; //flag changes to 1 if object already in lists
        for (int inList = 0; inList < matchingPoints.totalObjectsList; inList++) { //run through entire list of objects
            if (objectsFileStruct[isObject].id == matchingPoints.objectsList[inList].id) {
                //id is already in list, set variable to 1
                objectAlreadyInList = 1;
                //add 1 to total points with corresponding transform
                matchingPoints.objectsList[inList].totalCorrespondingPoints++;
            }
        }
        if (objectAlreadyInList == 0) { //transform not found in list
            matchingPoints.objectsList[matchingPoints.totalObjectsList].id = objectsFileStruct[isObject].id;
            matchingPoints.objectsList[matchingPoints.totalObjectsList].object_name = objectsFileStruct[isObject].object_name;
            matchingPoints.objectsList[matchingPoints.totalObjectsList].totalCorrespondingPoints++;
            matchingPoints.totalObjectsList++;
        }
        else if (objectAlreadyInList == 1) { //transform already in list
            //ignore, object already in struct
        }
    }
    //check to see if object has been detected from publish objects node
    getCorrespondingObjectFrame(isObject);
}

/**
 * Function adds all transforms with corresponding pc2 points to array
 *
 * @param parameter 'isObject' is the position of the object current being
 * iterated through findMatchingPoints
 */
void transformsFoundInPointcloudDistance(int isObject) {
    double cameraX = cameraTranslation.getOrigin().x();
    double cameraY = cameraTranslation.getOrigin().y();
    double cameraZ = cameraTranslation.getOrigin().z();
    double cameraXObjectDist = std::abs(cameraX - objectsFileStruct[isObject].point_x);
    double cameraYObjectDist = std::abs(cameraY - objectsFileStruct[isObject].point_y);
    if (DEBUG_transformsFoundInPointcloudDistance) {
        cout << "dist between camera and object is " << cameraXObjectDist << ", " << cameraYObjectDist << endl;
    }

    //if objects are distance away from camera
    if ((cameraXObjectDist < boundary.visualMaxBoundaryX) && (cameraYObjectDist < boundary.visualMaxBoundaryY)) {
        if (DEBUG_transformsFoundInPointcloudDistance_detections) {
            cout << objectsFileStruct[isObject].id << objectsFileStruct[isObject].object_name << " inside range" << endl;
        }
        if (matchingPoints.totalObjectsList == 0) { //set object id to first element in array
            matchingPoints.objectsList[matchingPoints.totalObjectsList].id = objectsFileStruct[isObject].id;
            matchingPoints.objectsList[matchingPoints.totalObjectsList].object_name = objectsFileStruct[isObject].object_name;
            matchingPoints.objectsList[matchingPoints.totalObjectsList].totalCorrespondingPoints++;
            matchingPoints.totalObjectsList++; //iterate to next item in array
        }
        else {
            int objectAlreadyInList = 0; //flag changes to 1 if object already in lists
            for (int inList = 0; inList < matchingPoints.totalObjectsList; inList++) { //run through entire list of objects
                if (objectsFileStruct[isObject].id == matchingPoints.objectsList[inList].id) {
                    //id is already in list, set variable to 1
                    objectAlreadyInList = 1;
                    //add 1 to total points with corresponding transform
                    matchingPoints.objectsList[inList].totalCorrespondingPoints++;
                }
            }
            if (objectAlreadyInList == 0) { //transform not found in list
                matchingPoints.objectsList[matchingPoints.totalObjectsList].id = objectsFileStruct[isObject].id;
                matchingPoints.objectsList[matchingPoints.totalObjectsList].object_name = objectsFileStruct[isObject].object_name;
                matchingPoints.objectsList[matchingPoints.totalObjectsList].totalCorrespondingPoints++;
                matchingPoints.totalObjectsList++; //iterate to next item in array
            }
            else if (objectAlreadyInList == 1) { //transform already in list
                //ignore, object already in struct
            }
        }
        //check to see if object has been detected from publish objects node
        getCorrespondingObjectFrame(isObject);
    }
    else {
        //ignoring objects further than boundary.visualMaxBoundary
    }
}

/**
 * Function runs through all points in pointcloud and
 * calculates if transform in close proximity
 *
 * @param parameter 'dpth' is ROS message of type pc2
 *        message belongs to sensor_msgs PointCloud2 constant pointer
 */
void findMatchingPoints(const sensor_msgs::PointCloud2::ConstPtr& dpth) {
    resetMatchingPoints(); //reset variables back to 0 for matching points struct
    int filterTransforms[fov.numberOfPixels];
    int totalFilterTransforms = 0;
    if (DEBUG_findMatchingPoints) {
        cout << "find matching points" << endl;
        cout << "number of pixels " << fov.numberOfPixels << endl;
    }
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*dpth, "x"); it != it.end(); ++it) {
        double pcloudX = it[0];
        double pcloudY = it[1];
        double pcloudZ = it[2];
        if (DEBUG_findMatchingPoints_rawValues) {
            //std::cout << it[0] << ", " << it[1] << ", " << it[2] << '\n';
            std::cout << pcloudX << ", " << pcloudY << ", " << pcloudZ << '\n';
        }
        //run through entire struct of objects
        for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
            double objectPointX = objectsFileStruct[isObject].point_x;
            double objectPointY = objectsFileStruct[isObject].point_y;
            double minObjectPointX = objectPointX - boundary.pointBoundaryX;
            double maxObjectPointX = objectPointX + boundary.pointBoundaryX;
            double minObjectPointY = objectPointY - boundary.pointBoundaryY;
            double maxObjectPointY = objectPointY + boundary.pointBoundaryY;
            if ((pcloudX > minObjectPointX) &&
                (pcloudX < maxObjectPointX) &&
                (pcloudY > minObjectPointY) &&
                (pcloudY < maxObjectPointY)) {
                //transform detected close to pc2 point
                if (DEBUG_findMatchingPoints_detectedPoints) {
                    cout << "found " << objectsFileStruct[isObject].id << objectsFileStruct[isObject].object_name << endl;
                }
                //add corresponding transform to array
                transformsFoundInPointcloud(isObject);
            }
        }
    }
    //print out redetected objects in frame
    //set variables back to 0 for next pointcloud
    //make list of transforms with close points
    //make list of transforms with close points and has an associated detection
    //compare the both lists and work out which objects are missing
    //probably need a max distance used between transform and camera for visually detecting the object
}

void calculateMissingObjects() {
    //run through entire list of objects that should be detected
    //set a flag to 0
    //run through list of redetected objects
    //set a flag to 1 if ids are equal in both lists
    //run through entire list again, and pick out object flags that are still 0
    //objects of 0 are missing.
    for (int isDetection = 0; isDetection < matchingPoints.totalObjectsList; isDetection++) {
        int getDetectedObjectID = matchingPoints.objectsList[isDetection].id;
        for (int isRedetection = 0; isRedetection < matchingPoints.totalObjectsRedetected; isRedetection++) {
            int getRedetectedObjectID = matchingPoints.objectsRedetected[isRedetection].id;
            if (getDetectedObjectID == getRedetectedObjectID) {
                matchingPoints.objectsList[isDetection].foundFlag = 1;
            }
            else {
                //found flag remains at 0 in struct
            }
        }
    }
    //add objects with found flags equal 0 to non detections array
    for (int isDetection = 0; isDetection < matchingPoints.totalObjectsList; isDetection++) {
        int getDetectedObjectID = matchingPoints.objectsList[isDetection].id;
        std::string getDetectedObjectName = matchingPoints.objectsList[isDetection].object_name;
        if (matchingPoints.objectsList[isDetection].foundFlag == 0) {
            //add missing/non detected object to array
            matchingPoints.objectsNotRedetected[matchingPoints.totalObjectsNotRedetected].id = getDetectedObjectID;
            matchingPoints.objectsNotRedetected[matchingPoints.totalObjectsNotRedetected].object_name = getDetectedObjectName;
            matchingPoints.totalObjectsNotRedetected++;
        }
        else if (matchingPoints.objectsList[isDetection].foundFlag == 0) {
            //ignore objects that have already been redetected
        }
    }
}

void printAllObjects() {
    if (DEBUG_printAllObjects) {
        tofToolBox->printSeparator(0);
        cout << "objectsList array: all objects that should be found" << endl;
        for (int isObject = 0; isObject < matchingPoints.totalObjectsList; isObject++) {
            cout <<
            matchingPoints.objectsList[isObject].id << ", " <<
            matchingPoints.objectsList[isObject].object_name << ", " <<
            matchingPoints.objectsList[isObject].totalCorrespondingPoints << endl;
        }
    }
}

void printRedetectedObjects() {
    if (DEBUG_printRedetectedObjects) {
        cout << "objectsRedetected array: all objects that have been redetected" << endl;
        for (int isObject = 0; isObject < matchingPoints.totalObjectsRedetected; isObject++) {
            cout <<
            matchingPoints.objectsRedetected[isObject].id << ", " <<
            matchingPoints.objectsRedetected[isObject].object_name << ", " <<
            matchingPoints.objectsRedetected[isObject].totalCorrespondingPoints << endl;
        }
    }
}

void printMissingObjects() {
    if (DEBUG_printMissingObjects) {
        cout << "objectsNotRedetected array: all objects that are missing, that should be detected" << endl;
        for (int isObject = 0; isObject < matchingPoints.totalObjectsNotRedetected; isObject++) {
            cout <<
            matchingPoints.objectsNotRedetected[isObject].id << ", " <<
            matchingPoints.objectsNotRedetected[isObject].object_name << ", " <<
            matchingPoints.objectsNotRedetected[isObject].totalCorrespondingPoints << endl;
        }
    }
}

void publishAllObjects() {
    wheelchair_msgs::missingObjects misObj;
    misObj.camera_timestamp = camera_timestamp;
    misObj.totalObjects = matchingPoints.totalObjectsList;
    for (int isObject = 0; isObject < matchingPoints.totalObjectsList; isObject++) {
        misObj.id.push_back(matchingPoints.objectsList[isObject].id);
        misObj.object_name.push_back(matchingPoints.objectsList[isObject].object_name);
        misObj.totalCorrespondingPoints.push_back(matchingPoints.objectsList[isObject].totalCorrespondingPoints);
    }
    ptr_pub_objectsList->publish(misObj);
}

void publishRedetectedObjects() {
    wheelchair_msgs::missingObjects misObj;
    misObj.camera_timestamp = camera_timestamp;
    misObj.totalObjects = matchingPoints.totalObjectsRedetected;
    for (int isObject = 0; isObject < matchingPoints.totalObjectsRedetected; isObject++) {
        misObj.id.push_back(matchingPoints.objectsRedetected[isObject].id);
        misObj.object_name.push_back(matchingPoints.objectsRedetected[isObject].object_name);
        misObj.totalCorrespondingPoints.push_back(matchingPoints.objectsRedetected[isObject].totalCorrespondingPoints);
    }
    ptr_pub_objectsRedetected->publish(misObj);
}

void publishMissingObjects() {
    wheelchair_msgs::missingObjects misObj;
    misObj.camera_timestamp = camera_timestamp;
    misObj.totalObjects = matchingPoints.totalObjectsNotRedetected;
    for (int isObject = 0; isObject < matchingPoints.totalObjectsNotRedetected; isObject++) {
        misObj.id.push_back(matchingPoints.objectsNotRedetected[isObject].id);
        misObj.object_name.push_back(matchingPoints.objectsNotRedetected[isObject].object_name);
        misObj.totalCorrespondingPoints.push_back(matchingPoints.objectsNotRedetected[isObject].totalCorrespondingPoints);
    }
    ptr_pub_objectsNotRedetected->publish(misObj);
}

/**
 * Callback function triggered by received ROS topic full list of objects
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void objectLocationsCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth) {
    //calculate field of view
    //calcualte the orientation of the robot, filter transforms within field of view of the robot
    //iterate through pointcloud and find nearest point to transform, probably in xy coordinate
    //see if transform exists within a range
    if (DEBUG_rosPrintSequence) {
        cout << "inside callback" << endl;
        //rosPrintSequence(dpth, obLoc);
    }
    getResolutionOnStartup(dpth); //get pointcloud image size
    getPointCloudTimestamp(dpth);
    //addObjectLocationsToStruct(obLoc);
    //getCameraTranslation();

    findMatchingPoints(dpth);
    calculateMissingObjects();
    //print array of objects
    printAllObjects();
    printRedetectedObjects();
    printMissingObjects();
    //publish array of objects
    publishAllObjects();
    publishRedetectedObjects();
    publishMissingObjects();
}

int main (int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    ros::init(argc, argv, "missing_objects");
    ros::NodeHandle n;

    ros::Rate rate(10.0);

    cache.setCacheSize(1000);
    //ros::Subscriber det_sub = n.subscribe("/wheelchair_robot/dacop/publish_object_locations/detected_objects", 1000, detectedObjectsCallback);

    //set detected objects through separate thread
    /*ros::NodeHandle nh_detectedObjects;
    ros::CallbackQueue callback_queue_detectedObjects;
    nh_detectedObjects.setCallbackQueue(&callback_queue_detectedObjects);
    ros::Subscriber detected_objects_sub = nh_detectedObjects.subscribe(
                                            "/wheelchair_robot/dacop/publish_object_locations/detected_objects",
                                            1000,
                                            detectedObjectsCallback);
    std::thread spinner_thread_delay([&callback_queue_detectedObjects]() {
        ros::SingleThreadedSpinner spinner_detectedObjects;
        spinner_detectedObjects.spin(&callback_queue_detectedObjects);
    });*/

    //add list of all objects to separate thread
    /*ros::NodeHandle nh_objectsList;
    ros::CallbackQueue callback_queue_objectsList;
    nh_objectsList.setCallbackQueue(&callback_queue_objectsList);
    ros::Subscriber objects_sub = nh_objectsList.subscribe(
                                            "/wheelchair_robot/dacop/publish_object_locations/objects",
                                            1000,
                                            addObjectLocationsToStruct);
    std::thread spinner_objectsList([&callback_queue_objectsList]() {
        ros::SingleThreadedSpinner spinner_objectsList;
        spinner_objectsList.spin(&callback_queue_objectsList);
    });*/

    ros::Subscriber pc2_sub = n.subscribe<sensor_msgs::PointCloud2>("/wheelchair_robot/point_cloud_map", 1000, objectLocationsCallback);

    ros::CallbackQueue detected_objects_queue;
    ros::CallbackQueue listed_objects_queue;

    ros::NodeHandle do_n;
    ros::NodeHandle lo_n;

    do_n.setCallbackQueue(&detected_objects_queue);
    lo_n.setCallbackQueue(&listed_objects_queue);

    ros::Subscriber detected_objects_sub = do_n.subscribe<wheelchair_msgs::objectLocations>(
                                            "/wheelchair_robot/dacop/publish_object_locations/detected_objects",
                                            1000,
                                            detectedObjectsCallback);
    ros::Subscriber listed_objects_sub = lo_n.subscribe<wheelchair_msgs::objectLocations>(
                                            "/wheelchair_robot/dacop/publish_object_locations/objects",
                                            1000,
                                            addObjectLocationsToStruct);

    // Create AsyncSpinner, run it on all available cores and make it process custom callback queue
    detected_objects_spinner.reset(new ros::AsyncSpinner(0, &detected_objects_queue));
    listed_objects_spinner.reset(new ros::AsyncSpinner(0, &listed_objects_queue));

    //get transformed pointcloud
    //message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "/zed/zed_node/point_cloud/cloud_registered", 1000);
    //get transformed pointcloud
    /*message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "/wheelchair_robot/point_cloud_map", 1000);
    //get mobilenet objects detected
    message_filters::Subscriber<wheelchair_msgs::objectLocations> objects_sub(n, "/wheelchair_robot/dacop/publish_object_locations/objects", 1000);
    //approximately sync the topic rate
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, wheelchair_msgs::objectLocations> MySyncPolicy;
    //set sync policy
    message_filters::Synchronizer<MySyncPolicy> depth_sync(MySyncPolicy(20), depth_sub, objects_sub);
    //set callback for synced topics
    depth_sync.registerCallback(boost::bind(&objectLocationsCallback, _1, _2));*/

    //publish all objects that should be detected within frame
    ros::Publisher pub_objectsList = n.advertise<wheelchair_msgs::missingObjects>("/wheelchair_robot/dacop/missing_objects/all", 1000);
    ptr_pub_objectsList = &pub_objectsList; //pointer to publish all objects that should be detected

    //publish objects that have been redetected within frame
    ros::Publisher pub_objectsRedetected = n.advertise<wheelchair_msgs::missingObjects>("/wheelchair_robot/dacop/missing_objects/redetected", 1000);
    ptr_pub_objectsRedetected = &pub_objectsRedetected; //pointer to publish all objects that should be detected

    //publish objects that are missing from the frame
    ros::Publisher pub_objectsNotRedetected = n.advertise<wheelchair_msgs::missingObjects>("/wheelchair_robot/dacop/missing_objects/missing", 1000);
    ptr_pub_objectsNotRedetected = &pub_objectsNotRedetected; //pointer to publish objects that are missing

    while(ros::ok()) {
        //tofToolBox->sayHello(); //test function for tof toolbox

        // Clear old callback from the queue
        //detected_objects_queue.clear();
        //listed_objects_queue.clear();
        // Start the spinner
        detected_objects_spinner->start();
        listed_objects_spinner->start();
        //ROS_INFO("detected_objects Spinner enabled");

        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
    detected_objects_spinner->stop();
    listed_objects_spinner->stop();
    ROS_INFO("detected_objects_Spinner disabled");
    detected_objects_spinner.reset();
    listed_objects_spinner.reset();
    // Wait for ROS threads to terminate
    ros::waitForShutdown();
}
