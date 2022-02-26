/*
 * missing_objects.cpp
 * wheelchair_context
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
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

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <math.h>

using namespace std;

static const int DEBUG_getResolutionOnStartup = 0;
static const int DEBUG_rosPrintSequence = 0;
static const int DEBUG_addObjectLocationsToStruct = 0;
static const int DEBUG_getPointCloudTimestamp = 0;
static const int DEBUG_objectsInFielfOfView = 0;
static const int DEBUG_detectedObjectsCallback = 0;
static const int DEBUG_findMatchingPoints = 0;
static const int DEBUG_findMatchingPoints_rawValues = 0;
static const int DEBUG_findMatchingPoints_detectedPoints = 0;
static const int DEBUG_getCorrespondingObjectFrame_redetectedObjects = 0;
static const int DEBUG_printAllObjects = 1;
static const int DEBUG_printRedetectedObjects = 1;
static const int DEBUG_printMissingObjects = 1;
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
struct Objects *objectsInFielfOfViewStruct[100000];
/*
struct Objects detectedObjects[10000]; //array for storing detected object data
int totalObjectsDetected = 0; //total objects inside struct
struct Objects objectsRedetected[1000]; //store objects that have been redetected in struct
int totalObjectsRedetected = 0; //total objects inside redetected objects struct
struct Objects objectsNotRedetected[1000]; //store objects that should have been redetected
int totalObjectsNotRedetected = 0; //total objects not redetected by node
*/


struct Objects *matchingsPoints[100000];

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
    int totalLocalObjectsRedetected = 0;
    int localObjectsRedetected[1000];

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

ros::Publisher *ptr_pub_objectsList;
ros::Publisher *ptr_pub_objectsRedetected;
ros::Publisher *ptr_pub_objectsNotRedetected;

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

void rosPrintSequence(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::objectLocations::ConstPtr& obLoc) {
    tofToolBox->printSeparator(0);
    cout << "depth header seq:  " << dpth->header.seq << endl;
    cout << "object header seq: " << obLoc->header.seq << endl;
    tofToolBox->printSeparator(0);
}

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

int objectsInFielfOfView() {
    /*object_on_world = PoseStamped()
    object_on_world.header.frame_id = 'map'
    object_on_world.pose.orientation.w = 1.0
    object_on_world.pose.position.x = object_x_pos
    object_on_world.pose.position.y = object_y_pos
    object_on_world.pose.position.z = object_z_pos

    transform = self.tf_buffer.lookup_transform('camera_depth_optical_frame', 'map', rospy.Time(0), rospy.Duration(1.0))
    object_on_camera = tf2_geometry_msgs.do_transform_pose(object_on_world, transform) #point on camera_depth_optical_frame*/
/*
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
        //geometry_msgs::PoseStamped objectOnWorld;

        //objectOnWorld.header.frame_id = "map"; //assign the pose 'map' frame
        //objectOnWorld.pose.orientation.w = objectsFileStruct[isObject].quat_w; //assign rotation quat w
        //objectOnWorld.pose.position.x = objectsFileStruct[isObject].point_x;
        //objectOnWorld.pose.position.y = objectsFileStruct[isObject].point_y;
        //objectOnWorld.pose.position.z = objectsFileStruct[isObject].point_z;

        //std::string getObjectID = to_string(objectsFileStruct[isObject].id);
        //std::string getObjectName = objectsFileStruct[isObject].object_name;
        //std::string DETframename = "/" + getObjectName + getObjectName;



        try {
            tf::StampedTransform cameraTranslation;
            ptrListener->waitForTransform("zed_camera_center", "/map", camera_timestamp, ros::Duration(1.0)); //wait a few seconds for ROS to respond
            ptrListener->lookupTransform("zed_camera_center", "/map", camera_timestamp, cameraTranslation); //lookup translation of object from map frame
            // try {
            //     tf::StampedTransform objectTranslation;
            //     ptrListener->waitForTransform(DETframename, cameraTranslation, camera_timestamp, ros::Duration(1.0)); //wait a few seconds for ROS to respond
            //     ptrListener->lookupTransform(DETframename, cameraTranslation, camera_timestamp, objectTranslation); //lookup translation of object from map frame
            // }
            // catch (tf::TransformException ex){
            //     cout << "Couldn't get base_link to map translation..." << endl; //catchment function if it can't get a translation from the map
            //     ROS_ERROR("%s",ex.what()); //print error
            //     ros::Duration(1.0).sleep();
            // }
        }
        catch (tf::TransformException ex){
            cout << "Couldn't get base_link to map translation..." << endl; //catchment function if it can't get a translation from the map
            ROS_ERROR("%s",ex.what()); //print error
            ros::Duration(1.0).sleep();
        }
    }
*/





/*
cv::Point2d PinholeCameraModel::project3dToPixel(const cv::Point3d& xyz) const
{
    assert( initialized() );
    assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane

    // [U V W]^T = P * [X Y Z 1]^T
    // u = U/W
    // v = V/W
    cv::Point2d uv_rect;
    uv_rect.x = (fx()*xyz.x + Tx()) / xyz.z + cx();
    uv_rect.y = (fy()*xyz.y + Ty()) / xyz.z + cy();
    return uv_rect;
}
*/

    cout << "in field of view function" << endl;
    tf::StampedTransform cameraTranslation;
    try {
        cout << "in try loop" << endl;
        ptrListener->waitForTransform("/map", "zed_camera_center", camera_timestamp, ros::Duration(3.0)); //wait a few seconds for ROS to respond
        ptrListener->lookupTransform("/map", "zed_camera_center", camera_timestamp, cameraTranslation); //lookup translation of object from map frame

        if ((cameraTranslation.getOrigin().z() < fov.minDepth) || (cameraTranslation.getOrigin().z() > fov.maxDepth)) {
            //objects behind the camera, with a distance minor/higher than the accepted are not considered in the field of fiew
        }
        else {
            double yangle = atan(cameraTranslation.getOrigin().y() / cameraTranslation.getOrigin().z()); //radians
            double xangle = atan(cameraTranslation.getOrigin().x() / cameraTranslation.getOrigin().z()); //radians

            double absX = abs(fov.fovx / 2 * M_PI / 180);
            double absY = abs(fov.fovy / 2 * M_PI / 180);
            if ((xangle < absX) && (yangle < absY)) {
                cout << "transform inside FOV" << endl;
            }
            else {
                cout << "transform not in FOV" << endl;
            }
        }
    }
    catch (tf::TransformException ex){
        cout << "Couldn't get base_link to map translation..." << endl; //catchment function if it can't get a translation from the map
        ROS_ERROR("%s",ex.what()); //print error
        ros::Duration(1.0).sleep();
    }

    //notes:
/*
    tf::StampedTransform translation; //initiate translation for transform object
    try {
        //ptrListener->waitForTransform("/map", DETframename, camera_timestamp, ros::Duration(3.0)); //wait a few seconds for ROS to respond
        //ptrListener->lookupTransform("/map", DETframename, camera_timestamp, translation); //lookup translation of object from map frame
    }
    catch (tf::TransformException ex){
        cout << "Couldn't get translation..." << endl; //catchment function if it can't get a translation from the map
        ROS_ERROR("%s",ex.what()); //print error
        ros::Duration(1.0).sleep();
    }*/
    return 0;
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
    cout << "number of pixels " << fov.numberOfPixels << endl;
    if (DEBUG_findMatchingPoints) {
        cout << "find matching points" << endl;
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
                //check to see if object has been detected from publish objects node
                getCorrespondingObjectFrame(isObject);
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

/*void findMatchingPoints(const sensor_msgs::PointCloud2::ConstPtr& dpth) {
    cout << "find matching points" << endl;
    int localObjectsRedetected[1000];
    int localObjectsRedetectedCounter = 0;
    int totalLocalObjectsRedetected = 0;
    int localObjectsNotRedetected[1000];
    int localObjectsNotRedetectedCounter = 0;
    int totalLocalObjectsNotRedetected = 0;

    for (sensor_msgs::PointCloud2ConstIterator<float> it(*dpth, "x"); it != it.end(); ++it) {
        // TODO: do something with the values of x, y, z
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
                                cout << objectsFileStruct[isObject].object_name << " HAS BEEN REDETECTED!" << endl;
                                if (totalLocalObjectsRedetected == 0) {
                                    //add first element to array
                                    //assign object id to local array
                                    localObjectsRedetected[totalLocalObjectsRedetected] = objectsFileStruct[isObject].id;
                                    totalLocalObjectsRedetected++; //iterate to next element in array

                                }
                                else {
                                    //run through for loop to see if id has already been detected
                                    int objectAlreadyInStruct = 0;
                                    for (int localDet = 0; localDet < totalLocalObjectsRedetected; localDet++) {
                                        if (objectsFileStruct[isObject].id == localObjectsRedetected[totalLocalObjectsRedetected]) {
                                            //id is already in struct
                                            objectAlreadyInStruct = 1;
                                        }
                                    }
                                    if (objectAlreadyInStruct == 0) {
                                        //object needs adding to objects redetected struct
                                        localObjectsRedetected[totalLocalObjectsRedetected] = objectsFileStruct[isObject].id;
                                        totalLocalObjectsRedetected++; //iterate to next element in array
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
                    cout << "Avoided NULL pointer" << endl;
                    //point is near object transform, but no dnn detection message
                    //no detection message received, assume object transform with corresponding points are no longer present

                }
            }
            else {
                //transform not detected close to point
            }
        }
    }
}*/

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
void objectLocationsCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::objectLocations::ConstPtr& obLoc) {
    //calculate field of view
    //calcualte the orientation of the robot, filter transforms within field of view of the robot
    //iterate through pointcloud and find nearest point to transform, probably in xy coordinate
    //see if transform exists within a range
    if (DEBUG_rosPrintSequence) {
        cout << "inside callback" << endl;
        rosPrintSequence(dpth, obLoc);
    }
    getResolutionOnStartup(dpth); //get pointcloud image size
    getPointCloudTimestamp(dpth);
    addObjectLocationsToStruct(obLoc);

    //objectsInFielfOfView(); //need to try and get this working
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

    //full list of objects
    //ros::Subscriber detected_objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 1000, detectedObjectsCallback);

    cache.setCacheSize(1000);
    ros::Subscriber det_sub = n.subscribe("/wheelchair_robot/dacop/publish_object_locations/detected_objects", 1000, detectedObjectsCallback);


    //get transformed pointcloud
    //message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "/zed/zed_node/point_cloud/cloud_registered", 1000);
    //get transformed pointcloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "/wheelchair_robot/point_cloud_map", 1000);
    //get mobilenet objects detected
    message_filters::Subscriber<wheelchair_msgs::objectLocations> objects_sub(n, "wheelchair_robot/dacop/publish_object_locations/objects", 1000);
    //approximately sync the topic rate
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, wheelchair_msgs::objectLocations> MySyncPolicy;
    //set sync policy
    message_filters::Synchronizer<MySyncPolicy> depth_sync(MySyncPolicy(20), depth_sub, objects_sub);
    //set callback for synced topics
    depth_sync.registerCallback(boost::bind(&objectLocationsCallback, _1, _2));

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
        tf::TransformListener listener;
        ptrListener = &listener; //set to global pointer - to access from another function

        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
}
