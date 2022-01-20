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

using namespace std;

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

ros::Publisher pcl_pub;

/**
 * Callback function triggered by received ROS topic
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void objectsFullListCallback(const wheelchair_msgs::objectLocations obLoc) {
    int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg
    totalObjectsFileStruct = totalObjectsInMsg; //set message total objects to total objects in file struct
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //iterate through entire msg topic array
        objectsFileStruct[isObject].id = obLoc.id[isObject]; //assign object id to struct
        objectsFileStruct[isObject].object_name = obLoc.object_name[isObject]; //assign object name to struct
        objectsFileStruct[isObject].object_confidence = obLoc.object_confidence[isObject]; //assign object confidence to struct

        objectsFileStruct[isObject].point_x = obLoc.point_x[isObject]; //assign object vector point x to struct
        objectsFileStruct[isObject].point_y = obLoc.point_y[isObject]; //assign object vector point y to struct
        objectsFileStruct[isObject].point_z = obLoc.point_z[isObject]; //assign object vector point z to struct

        objectsFileStruct[isObject].quat_x = obLoc.quat_x[isObject]; //assign object quaternion x to struct
        objectsFileStruct[isObject].quat_y = obLoc.quat_y[isObject]; //assign object quaternion y to struct
        objectsFileStruct[isObject].quat_z = obLoc.quat_z[isObject]; //assign object quaternion z to struct
        objectsFileStruct[isObject].quat_w = obLoc.quat_w[isObject]; //assign object quaternion w to struct
    }
}

int verifyFieldOfView() {
    //add code here
    return 0;
}


void objectLocationsCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::objectLocations::ConstPtr& obLoc) {
   //calculate field of view
   //calcualte the orientation of the robot, filter transforms within field of view of the robot
   //iterate through pointcloud and find nearest point to transform, probably in xy coordinate
   //see if transform exists within a range
}

int main (int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    ros::init(argc, argv, "missing_objects");
    ros::NodeHandle n;

    ros::Rate rate(10.0);

    //full list of objects
    ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 1000, objectsFullListCallback);

    while(ros::ok()) {
        //tofToolBox->sayHello(); //test function for tof toolbox
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
}
