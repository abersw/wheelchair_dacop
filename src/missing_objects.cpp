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

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <math.h>

using namespace std;

static const int DEBUG_addObjectLocationsToStruct = 0;
static const int DEBUG_getPointCloudTimestamp = 0;
static const int DEBUG_objectsInFielfOfView = 1;
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
struct Objects *objectsInFielfOfViewStruct[100000];

ros::Time camera_timestamp;
double camera_timestamp_sec;

struct FOV {
    double minDepth = 0.1;
    double maxDepth = 5;
    //https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-
    //HD-720p
    int fovx = 85; //horizontal field of view in deg
    int fovy = 54; //vertical field of view in deg
};
struct FOV fov;

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


    tf::StampedTransform cameraTranslation;
    try {
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

    tf::StampedTransform translation; //initiate translation for transform object
    try {
        //ptrListener->waitForTransform("/map", DETframename, camera_timestamp, ros::Duration(3.0)); //wait a few seconds for ROS to respond
        //ptrListener->lookupTransform("/map", DETframename, camera_timestamp, translation); //lookup translation of object from map frame
    }
    catch (tf::TransformException ex){
        cout << "Couldn't get translation..." << endl; //catchment function if it can't get a translation from the map
        ROS_ERROR("%s",ex.what()); //print error
        ros::Duration(1.0).sleep();
    }
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
