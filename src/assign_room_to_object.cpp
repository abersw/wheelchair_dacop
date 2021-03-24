/*
 * assign_room_to_object.cpp
 * wheelchair_dacop
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
 * 
 * This needs an input from the user - this is the kitchen
 * Detected objects will then be allocated a room
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include "wheelchair_msgs/objectLocations.h"
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

const int DEBUG_createFile = 1;
const int DEBUG_main = 1;

struct Objects {
    int object_id;
    string object_name;
    int room_id;
    string room_name;
};

std::string userRoomName;

//list of file locations
std::string wheelchair_dump_loc;
std::string dump_dacop_loc = "/dump/dacop/";
std::string rooms_list_name = "rooms.list"; //file with list of rooms
std::string rooms_list_loc; //full path for rooms list
std::string rooms_dacop_name = "rooms.dacop"; //file with objects associated with rooms
std::string rooms_dacop_loc; //full path for rooms dacop file

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

/**
 * Does the wheelchair_dump package exist in the workspace?
 * If it's missing, close down the node safely
 */
void doesWheelchairDumpPkgExist() {
	if (ros::package::getPath("wheelchair_dump") == "") {
		cout << "FATAL:  Couldn't find package 'wheelchair_dump' \n";
		cout << "FATAL:  Closing training_context node. \n";
		printSeparator(1);
		ros::shutdown();
		exit(0);
	}
}

/**
 * Function to check if file exists in the 'fileName' path, if it doesn't exist create a new one
 *
 * @param pass the path and file name to be created called 'fileName'
 * @return return '1' if file already exists, return '0' if file was missing and has been created
 */
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

/**
 * Take room list file and add it to struct array for processing later 
 *
 * @param parameter fileName is the path of the room.list file
 */
void roomListToStruct(std::string fileName) {
    //add stuff here
}

/**
 * Main callback function triggered by received ROS topic 
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg ca
 */
void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg
}

/**
 * Last function to save all struct data into files, ready for using on next startup 
 */
void saveAllFiles() {
    cout << "saving all files" << endl;
}

/**
 * Main function that contains ROS info, subscriber callback trigger and while loop to get room name
 *
 * @param argc - number of arguments
 * @param argv - content of arguments
 * @return 0 - end of program
 */
int main (int argc, char **argv) {
    //add code here
    //notes:
    //take UID from publish_objects_location and pass it through here
    //when msg comes through with UID of object - append a room name to the object
    ros::init(argc, argv, "assign_room_to_object");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 10, objectLocationsCallback);
    //publish associated object rooms from publish object locations
    //publish all of objects and rooms struct
    std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
    doesWheelchairDumpPkgExist(); //check to see if dump package is present

	rooms_list_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_list_name; //concatenate vars to create location of rooms list
    createFile(rooms_list_loc); //check to see if file is present, if not create a new one

    rooms_dacop_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_dacop_name; //concatenate vars to create location of rooms dacop file
    createFile(rooms_dacop_loc);

    
    ros::Rate rate(10.0);
    while(ros::ok()) {
        n.getParam("/wheelchair_robot/param/user/room_name", userRoomName);
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
    saveAllFiles();
    return 0;
}