/*
 * publish_object_locations.cpp
 * wheelchair_dacop
 * version: 0.0.0 Majestic Maidenhair
 * Status: Gamma
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include "wheelchair_msgs/foundObjects.h"
#include "wheelchair_msgs/objectLocations.h"

//experimental
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <fstream>
#include <iostream>



#include <sstream>
using namespace std;

const int DEBUG_doesWheelchairDumpPkgExist = 0;
const int DEBUG_createFile = 0;
const int DEBUG_calculateLines = 0;
const int DEBUG_publishObjectsStruct = 0;
const int DEBUG_objectLocationsCallback = 0;
const int DEBUG_main = 0;

struct Objects { //struct for publishing topic
    int id;
    string object_name;
    double object_confidence;
    double point_x;
    double point_y;
    double point_z;

    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
};
struct Objects objectsFileStruct[10000];
int totalObjectsFileStruct = 0;

std::string wheelchair_dump_loc;

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

//calculate lines from files
int calculateLines(std::string fileName) {
	if (DEBUG_calculateLines) {
		cout << "DEBUG: calculateLines()\n";
	}
	ifstream FILE_COUNTER(fileName);
	std::string getlines;
	int returnCounter = 0;
	while (getline (FILE_COUNTER, getlines)) {
        returnCounter++;
        // Output the text from the file
        //cout << getlines;
        //cout << "\n";
	}
	FILE_COUNTER.close();
    if (DEBUG_calculateLines) {
        cout << returnCounter << endl;
    }
	return returnCounter;
}

void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    totalObjectsFileStruct = obLoc.totalObjects;

    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
        objectsFileStruct[isObject].id = obLoc.id[isObject];
        objectsFileStruct[isObject].object_name = obLoc.object_name[isObject];
        objectsFileStruct[isObject].object_confidence = obLoc.object_confidence[isObject];
        objectsFileStruct[isObject].point_x = obLoc.point_x[isObject];
        objectsFileStruct[isObject].point_y = obLoc.point_y[isObject];
        objectsFileStruct[isObject].point_z = obLoc.point_z[isObject];
        objectsFileStruct[isObject].quat_x = obLoc.quat_x[isObject];
        objectsFileStruct[isObject].quat_y = obLoc.quat_y[isObject];
        objectsFileStruct[isObject].quat_z = obLoc.quat_z[isObject];
        objectsFileStruct[isObject].quat_w = obLoc.quat_w[isObject];

        if (DEBUG_objectLocationsCallback) {
            //add debug lines here
            cout << "object struct" << endl;
            cout << objectsFileStruct[isObject].id << ", " << objectsFileStruct[isObject].object_name << ", " << objectsFileStruct[isObject].object_confidence << endl;
            cout << objectsFileStruct[isObject].point_x << ", " << objectsFileStruct[isObject].point_y << ", " << objectsFileStruct[isObject].point_z << endl;
            cout << objectsFileStruct[isObject].quat_x << ", " << objectsFileStruct[isObject].quat_y << ", " << objectsFileStruct[isObject].quat_z << ", " << objectsFileStruct[isObject].quat_w << endl;
            printSeparator(0);
        }
    }
}

void publishObjectsStruct() {
    //add code here to publish struct continuously
    wheelchair_msgs::objectLocations obLoc;
    //publish all objects inside struct
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //iterate through entire struct

        int objectID = objectsFileStruct[isObject].id;
        std::string objectName = objectsFileStruct[isObject].object_name;

        static tf::TransformBroadcaster br;
        std::string OBframename = std::to_string(objectID) + objectName;

        //turn msg to pose
        tf::Transform mapTransform;
        //create map transform from map to object frame
        mapTransform.setOrigin( tf::Vector3(objectsFileStruct[isObject].point_x, objectsFileStruct[isObject].point_y, objectsFileStruct[isObject].point_z) );
        tf::Quaternion mapQuaternion(objectsFileStruct[isObject].quat_x, objectsFileStruct[isObject].quat_y, objectsFileStruct[isObject].quat_z, objectsFileStruct[isObject].quat_w);
        mapTransform.setRotation(mapQuaternion);
        br.sendTransform(tf::StampedTransform(mapTransform, ros::Time::now(), "map", OBframename));
        //end the map frame object publishing
        if (DEBUG_publishObjectsStruct) {
            cout << "publishing map frame" << endl;
        }
    }
}

void objectsStructToList(std::string objects_file_loc) {
    //add struct to list file here
    ofstream FILE_WRITER;
	FILE_WRITER.open(objects_file_loc);
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
        FILE_WRITER << objectsFileStruct[isObject].id << "," << objectsFileStruct[isObject].object_name << "," << objectsFileStruct[isObject].object_confidence << "," <<
        objectsFileStruct[isObject].point_x << "," << objectsFileStruct[isObject].point_y << "," << objectsFileStruct[isObject].point_z << "," <<
        objectsFileStruct[isObject].quat_x << "," << objectsFileStruct[isObject].quat_y << "," << objectsFileStruct[isObject].quat_z << "," << objectsFileStruct[isObject].quat_w << "\n";
    }
    FILE_WRITER.close();
    cout << "finished saving function" << endl;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_object_locations");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("wheelchair_robot/object_locations/objects", 10, objectLocationsCallback);
    //other subscribers can be added to modify the central objects struct to list
    ros::Rate rate(10.0);

    wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
    std::string objects_file_loc = wheelchair_dump_loc + "/dump/dacop/objects.dacop";

    doesWheelchairDumpPkgExist();
    int objectsListExists = createFile(objects_file_loc);
    
    while(ros::ok()) {

        publishObjectsStruct();
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
    cout << "saving struct to list" << endl;
    objectsStructToList(objects_file_loc);
    return 0;
}