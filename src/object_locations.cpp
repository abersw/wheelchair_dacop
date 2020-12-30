#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include <ros/package.h> //find ROS packages, needs roslib dependency
#include "wheelchair_msgs/objectLocations.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

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
#include <fstream>
#include <iostream>



#include <sstream>
using namespace std;
//using namespace message_filters;
//using namespace sensor_msgs;


//DEBUG LINES - set variable to 1 to enable, 0 to disable
const int DEBUG_doesWheelchairDumpPkgExist = 1;
const int DEBUG_createFile = 1;

std::string wheelchair_dump_loc;

struct Objects { //struct for publishing topic
    int id;
    string object_name;
    float point_x;
    float point_y;
    float point_z;

    float rotation_r;
    float rotation_p;
    float rotation_y;
};
int totalObjects;


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

//create a file
int createFile(std::string fileName) { //if this doesn't get called, no file is created
    if (DEBUG_createFile) {
    	printf("DEBUG: createFile()\n");
    }
	std::ifstream fileExists(fileName);

	if (fileExists.good() == 1) {
		//File exists
        if (DEBUG_createFile) {
    		printf("Weighting file exists\n");
        }
		//cout << fileName;
		return 1;
	}
	else {
		//File doesn't exist
        if (DEBUG_createFile) {
    		printf("Weighting file doesn't exist\n");
	    	printf("creating new file\n");
        }
		ofstream NEW_FILE (fileName);
		NEW_FILE.close();
		//cout << fileName;
		return 0;
	}
}

void objectsListToStruct(std::string objects_file_loc) {
    //add list to stuct - test this first before callback
}

void objectsDetectedCallback(const wheelchair_msgs::objectLocations objects_msg) {
    //stuff here on each callback
    //if object isn't detected in room - reduce object influence (instead of deleting?)

    /*
    //# Object location data from map

    Header header
    int16[] id
    string[] object_name

    float64[] point_x
    float64[] point_y
    float64[] point_z

    float64[] rotation_r
    float64[] rotation_p
    float64[] rotation_y

    int16 totalObjects*/

    int totalObjects = objects_msg.totalObjects;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "object_depth");
    ros::NodeHandle n;

    doesWheelchairDumpPkgExist();

    ros::Rate rate(10.0);
    ros::Subscriber sub = n.subscribe("wheelchair_robot/object_depth/detected_objects", 10, objectsDetectedCallback);
    wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
    std::string objects_file_loc = wheelchair_dump_loc + "/dump/dacop/objects.dacop";

    int objectsListExists = createFile(objects_file_loc); //create room list

    objectsListToStruct(objects_file_loc);


    //set global variable for file/database
    //if does not exist - create one
    //if using a database and table does not exist - create one
    
    if (ros::isShuttingDown()) {
        //close things safely
    }
    cout << "spin \n";
    ros::spin();
    rate.sleep();

    return 0;
}