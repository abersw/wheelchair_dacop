/*
 * assign_room_to_object.cpp
 * wheelchair_dacop
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
 * 
 * This needs an input from the user - this is the kitchen
 * Detected objects will then be allocated a room
 * 
 * Change room name parameter to topic - then it won't need to continuously check if room exists
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include "std_msgs/String.h" //for room name topic
#include "wheelchair_msgs/objectLocations.h"
#include "wheelchair_msgs/roomToObjects.h"
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

const int DEBUG_createFile = 1;
const int DEBUG_roomListToStruct = 1;
const int DEBUG_roomNameCallback = 1;
const int DEBUG_main = 0;

struct Rooms {
    int room_id;
    string room_name;
};
int totalRoomsFileStruct = 0;
struct Rooms roomsFileStruct[1000];


struct Objects {
    int object_id;
    string object_name;

    int room_id;
    string room_name;
};
int totalObjectsFileStruct = 0;
struct Objects objectsFileStruct[100000]; //array for storing all object and room data

std::string userRoomName;
int currentRoomID;
std::string currentRoomName;

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
    if (DEBUG_roomListToStruct) {
        cout << "DEBUG_roomListToStruct" << endl;
    }
    //id, room name
    std::string objectsDelimiter = ","; //delimiter character is comma
	ifstream FILE_READER(fileName); //open file
    int roomNumber = 0; //iterate on each object
    if (FILE_READER.peek() == std::ifstream::traits_type::eof()) {
        //don't do anything if next character in file is eof
        cout << "file is empty" << endl;
    }
    else {
        std::string line;
        while (getline(FILE_READER, line)) { //go through line by line
            int lineSection = 0; //var for iterating through serialised line
            int pos = 0; //position of delimiter in line
            std::string token;
            while ((pos = line.find(objectsDelimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                //std::cout << token << std::endl;
                line.erase(0, pos + objectsDelimiter.length());
                //deserialise the line sections below:
                if (lineSection == 0) {
                    roomsFileStruct[roomNumber].room_id = roomNumber; //set id of room back to 0
                }
                lineSection++;
            }
            roomsFileStruct[roomNumber].room_name = line; //set end of line to room name
            if (DEBUG_roomListToStruct) {
                cout << roomsFileStruct[roomNumber].room_id << "," << roomsFileStruct[roomNumber].room_name << endl;
            }
            roomNumber++;
        }
    }
    totalRoomsFileStruct = roomNumber;
}

/**
 * Main callback function triggered by received ROS topic 
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg ca
 */
void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg
    for (int isObject = 0; isObject < totalObjectsInMsg; isObject++) {

        if (totalObjectsFileStruct == 0) { //can't start for loop if struct is empty - so add some initial data
            //add object to struct
        }
    }
}

void roomNameCallback(const std_msgs::String roomNameMsg) {
    if (DEBUG_roomNameCallback) {
        cout << "DEBUG_roomNameCallback" << endl;
    }
    std::string roomName_msg = roomNameMsg.data;
    int roomDetected = 0;

    int tempRoomID;
    std::string tempRoomName;

    if (DEBUG_roomNameCallback) {
        cout << "user input from topic is " << roomName_msg << endl; //this is working - something is going wrong further down in this function...
    }
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) { //run through entire rooms struct
        if (roomsFileStruct[isRoom].room_name == roomName_msg) { //if room name in struct is equal to room name from topic
            roomDetected = 1; //Room is already in rooms struct
            tempRoomID = roomsFileStruct[isRoom].room_id;
            tempRoomName = roomsFileStruct[isRoom].room_name;
        }
        else {
            //don't set anything, the room detected var will remain at 0
        }
    }
    if (roomDetected) {
        currentRoomID = tempRoomID;
        currentRoomName = tempRoomName;
    }
    else {
        //add room not detected to struct
        roomsFileStruct[totalRoomsFileStruct].room_id = totalRoomsFileStruct;
        roomsFileStruct[totalRoomsFileStruct].room_name = roomName_msg;
        currentRoomID = roomsFileStruct[totalRoomsFileStruct].room_id;
        currentRoomName = roomsFileStruct[totalRoomsFileStruct].room_name;
    }
    if (DEBUG_roomNameCallback) {
        cout << "current room id is " << currentRoomID << endl;
        cout << "current room name is " << currentRoomName << endl;
    }
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
    std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
    doesWheelchairDumpPkgExist(); //check to see if dump package is present

	rooms_list_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_list_name; //concatenate vars to create location of rooms list
    createFile(rooms_list_loc); //check to see if file is present, if not create a new one

    rooms_dacop_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_dacop_name; //concatenate vars to create location of rooms dacop file
    createFile(rooms_dacop_loc);

    roomListToStruct(rooms_list_loc);
    ros::init(argc, argv, "assign_room_to_object");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 10, objectLocationsCallback);
    ros::Subscriber roomName_sub = n.subscribe("wheelchair_robot/user/room_name", 10, roomNameCallback);
    //publish associated object rooms from publish object locations
    //publish all of objects and rooms struct

    
    ros::Rate rate(10.0);
    while(ros::ok()) {
        //n.getParam("/wheelchair_robot/param/user/room_name", userRoomName);
        //cout << "room param is " << userRoomName << endl;
        
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spin();
        rate.sleep();
    }
    saveAllFiles();
    return 0;
}