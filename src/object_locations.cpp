/*
 * object_locations.cpp
 * wheelchair_dacop
 * version: 0.1.0 Majestic Maidenhair
 * Status: pre-Alpha
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
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
const int DEBUG_print_objectLocations_msg = 1;
const int DEBUG_main = 1;

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

void printObjectLocationsMsg(const wheelchair_msgs::objectLocations objects_msg, const int isObject) {
    printSeparator(0);
    cout << "ID: " << objects_msg.id[isObject] << endl;
    cout << "Name: " << objects_msg.object_name[isObject] << endl;

    cout << "Point_x: " << objects_msg.point_x[isObject] << endl;
    cout << "Point_y: " << objects_msg.point_y[isObject] << endl;
    cout << "Point_z: " << objects_msg.point_z[isObject] << endl;

    cout << "Rotation_r: " << objects_msg.rotation_r[isObject] << endl;
    cout << "Rotation_p: " << objects_msg.rotation_p[isObject] << endl;
    cout << "Rotation_y: " << objects_msg.rotation_y[isObject] << endl;
}

void objectsDetectedCallback(const wheelchair_msgs::objectLocations objects_msg) {
    //stuff here on each callback
    //if object isn't detected in room - reduce object influence (instead of deleting?)

    int totalObjects = objects_msg.totalObjects;
    for (int isObject = 0; isObject < totalObjects; isObject++) {
        if (DEBUG_print_objectLocations_msg) {
            printObjectLocationsMsg(objects_msg, isObject);
        }

        //turn to global map position
        //does it exist already?
        //yes - add context influence
        //no - add item
        //how to detect dissapearence? After period of time - reduce influence?

        //broadcast detected objects
        static tf::TransformBroadcaster br;
        std::string DETframename = "DET:" + objects_msg.object_name[isObject] + std::to_string(isObject);
        //turn msg to pose
        tf::Transform localTransform;
        tf::Transform mapTransform;
        //create local transform from zed camera to object
        localTransform.setOrigin( tf::Vector3(objects_msg.point_x[isObject], objects_msg.point_y[isObject], objects_msg.point_z[isObject]) );
        tf::Quaternion localQuaternion;
        localQuaternion.setRPY(objects_msg.rotation_r[isObject], objects_msg.rotation_p[isObject], objects_msg.rotation_y[isObject]);  //where r p y are fixed
        localTransform.setRotation(localQuaternion);
        br.sendTransform(tf::StampedTransform(localTransform, ros::Time::now(), "zed_left_camera_depth_link", DETframename));
        /*
        try {
            listener.lookupTransform("map", )
        }*/
    }
}



int main(int argc, char **argv) {
    //stuff to go here
    ros::init(argc, argv, "object_locations");

    ros::NodeHandle n;

    

    doesWheelchairDumpPkgExist();

    ros::Subscriber sub = n.subscribe("wheelchair_robot/object_depth/detected_objects", 10, objectsDetectedCallback);
    ros::Rate rate(10.0);
    wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
    std::string objects_file_loc = wheelchair_dump_loc + "/dump/dacop/objects.dacop";

    int objectsListExists = createFile(objects_file_loc); //create room list

    objectsListToStruct(objects_file_loc);


    //set global variable for file/database
    //if does not exist - create one
    //if using a database and table does not exist - create one
    
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