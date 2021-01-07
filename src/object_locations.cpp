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
const int DEBUG_doesWheelchairDumpPkgExist = 0;
const int DEBUG_createFile = 0;
const int DEBUG_calculateLines = 0;
const int DEBUG_objectsListToStruct = 0;
const int DEBUG_print_objectLocations_msg = 0;
const int DEBUG_doesObjectAlreadyExist = 1;
const int DEBUG_main = 1;

std::string wheelchair_dump_loc;


struct Objects { //struct for publishing topic
    int id;
    string object_name;
    float point_x;
    float point_y;
    float point_z;

    float quat_x;
    float quat_y;
    float quat_z;
    float quat_w;
};
struct Objects objectsFileStruct[10000];
int objectsFileTotalLines = 0;
int totalObjectsFileStruct = 0;
int objectTopologyThreshold = 0.5; //this should probably be a bounding box value...

tf::TransformListener *ptrListener;


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
	return returnCounter;
}

void objectsListToStruct(std::string objects_file_loc) {
    //add list to stuct - test this first before callback
    //contains transforms between map and object
    //id, object_name, point_x, point_y, point_z, quat_x, quat_y, quat_z, quat_w
    std::string objectsDelimiter = ",";
	ifstream FILE_READER(objects_file_loc);
	std::string line;
	int objectNumber = 0;
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
                objectsFileStruct[objectNumber].id = std::stoi(token);
            }
            else if (lineSection == 1) {
                objectsFileStruct[objectNumber].object_name = token;
            }
            else if (lineSection == 2) {
                objectsFileStruct[objectNumber].point_x = std::stof(token);
            }
            else if (lineSection == 3) {
                objectsFileStruct[objectNumber].point_y = std::stof(token);
            }
            else if (lineSection == 4) {
                objectsFileStruct[objectNumber].point_z = std::stof(token);
            }
            else if (lineSection == 5) {
                objectsFileStruct[objectNumber].quat_x = std::stof(token);
            }
            else if (lineSection == 6) {
                objectsFileStruct[objectNumber].quat_y = std::stof(token);
            }
            else if (lineSection == 7) {
                objectsFileStruct[objectNumber].quat_z = std::stof(token);
            }

            lineSection++;
        }
        //std::cout << line << std::endl;
        objectsFileStruct[objectNumber].quat_w = std::stof(line);
        cout << "sections in line " << lineSection << endl;
        if (DEBUG_objectsListToStruct) {
            cout << objectsFileStruct[objectNumber].id << "," << objectsFileStruct[objectNumber].object_name << endl;
            cout << objectsFileStruct[objectNumber].point_x << ", " << objectsFileStruct[objectNumber].point_y << ", " << objectsFileStruct[objectNumber].point_z << endl;
            cout << objectsFileStruct[objectNumber].quat_x << ", " << objectsFileStruct[objectNumber].quat_y << ", " << objectsFileStruct[objectNumber].quat_z << ", " << objectsFileStruct[objectNumber].quat_w << endl;
            printSeparator(0);
        }
    }
    objectNumber++;
    totalObjectsFileStruct = objectNumber; //var to add number of objects in struct
}

void objectsStructToList(std::string objects_file_loc) {
    //add struct to list file here
    ofstream FILE_WRITER;
	FILE_WRITER.open(objects_file_loc);
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
        FILE_WRITER << objectsFileStruct[isObject].id << "," << objectsFileStruct[isObject].object_name << "," <<
        objectsFileStruct[isObject].point_x << "," << objectsFileStruct[isObject].point_y << "," << objectsFileStruct[isObject].point_z << "," <<
        objectsFileStruct[isObject].quat_x << "," << objectsFileStruct[isObject].quat_y << "," << objectsFileStruct[isObject].quat_z << "," << objectsFileStruct[isObject].quat_w << "\n";
    }
    FILE_WRITER.close();
    cout << "finished saving function" << endl;
}

void doesObjectAlreadyExist(std::string msg_object_name, std::string DETframename) {
    tf::StampedTransform translation;
    int addToTotalObjectsFileStruct = 0;
    try {
        ptrListener->waitForTransform("/map", DETframename, ros::Time(0), ros::Duration(3.0));
        ptrListener->lookupTransform("/map", DETframename, ros::Time(), translation);

        float translation_x = translation.getOrigin().x();
        float translation_y = translation.getOrigin().y();
        float translation_z = translation.getOrigin().z();
        float rotation_x = translation.getRotation().x();
        float rotation_y = translation.getRotation().y();
        float rotation_z = translation.getRotation().z();
        float rotation_w = translation.getRotation().w();

        if (DEBUG_doesObjectAlreadyExist) {
            printSeparator(0);
            cout << msg_object_name << endl;
            cout << translation_x << ", " << translation_y << ", " << translation_z << endl;
            cout << rotation_x << ", " << rotation_y << ", " << rotation_z << ", " << rotation_w << endl;
        }
        for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //iterate through entire struct
            //this value is from a fixed distance threshold - should be the bounding box size...
            float minPointThreshold_x = translation_x - objectTopologyThreshold; //make minimum x bound
            float maxPointThreshold_x = translation_x + objectTopologyThreshold; //make maximum x bound
            float minPointThreshold_y = translation_y - objectTopologyThreshold; //make minimum y bound
            float maxPointThreshold_y = translation_y + objectTopologyThreshold; //make maximum y bound

            if (((translation_x >= minPointThreshold_x) && (translation_x <= maxPointThreshold_x)) && //if transform is between x bound
                ((translation_y >= minPointThreshold_y) && (translation_y <= maxPointThreshold_y)) && //and is between y bound
                (msg_object_name == objectsFileStruct[isObject].object_name)) { //and is the same object
                cout << "object already in this location" << endl;
                //if there is already an object within the x and y dimension with the same name
                //do not add object to struct
                //so do nothing?
            }
            else {
                //add new object to struct
                cout << "added new object location" << endl;
                //add object to last position in struct
                /*objectsFileStruct[totalObjectsFileStruct + 1].id = totalObjectsFileStruct;
                objectsFileStruct[totalObjectsFileStruct + 1].object_name = msg_object_name;
                objectsFileStruct[totalObjectsFileStruct + 1].point_x = translation_x;
                objectsFileStruct[totalObjectsFileStruct + 1].point_y = translation_y;
                objectsFileStruct[totalObjectsFileStruct + 1].point_z = translation_z;
                objectsFileStruct[totalObjectsFileStruct + 1].quat_x = rotation_x;
                objectsFileStruct[totalObjectsFileStruct + 1].quat_y = rotation_y;
                objectsFileStruct[totalObjectsFileStruct + 1].quat_z = rotation_z;
                objectsFileStruct[totalObjectsFileStruct + 1].quat_w = rotation_w;*/
                addToTotalObjectsFileStruct = 1;
                //totalObjectsFileStruct++; //this line is causing the segmentation fault... fix me
            }
        }
        if (addToTotalObjectsFileStruct == 1) {
            totalObjectsFileStruct += 1;
        }
        else {
            //don't do anything
        }
    }
    catch (tf::TransformException ex){
        cout << "Couldn't get translation..." << endl;
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
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

        //start the temporary frame publishing
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
        //end the temporary frame publishing

        doesObjectAlreadyExist(objects_msg.object_name[isObject], DETframename);
    }
}



int main(int argc, char **argv) {
    //stuff to go here
    ros::init(argc, argv, "object_locations");

    ros::NodeHandle n;
    wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
    std::string objects_file_loc = wheelchair_dump_loc + "/dump/dacop/objects.dacop";
    while (ros::ok()) {
    tf::TransformListener listener;
    ptrListener = &listener;

    doesWheelchairDumpPkgExist();

    ros::Subscriber sub = n.subscribe("wheelchair_robot/object_depth/detected_objects", 10, objectsDetectedCallback);
    ros::Rate rate(10.0);
    

    int objectsListExists = createFile(objects_file_loc); //create room list
    objectsFileTotalLines = calculateLines(objects_file_loc);
    objectsListToStruct(objects_file_loc);


    //set global variable for file/database
    //if does not exist - create one
    //if using a database and table does not exist - create one

    //detecting missing objects
    //transform is a point, right?
    //scan through entire cloud to see if an individual point is within 0.5m and has an object label
    
    
    if (DEBUG_main) {
        cout << "spin \n";
    }
    ros::spin();
    rate.sleep();
    }
        //do something
        cout << "closing ROS node" << endl;
        objectsStructToList(objects_file_loc);
    return 0;
}