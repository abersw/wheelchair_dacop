/*
 * object_locations.cpp
 * wheelchair_dacop
 * version: 0.2.0 Majestic Maidenhair
 * Status: Alpha
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

//DEBUG LINES - set variable to 1 to enable, 0 to disable
const int DEBUG_doesWheelchairDumpPkgExist = 0;
const int DEBUG_createFile = 0;
const int DEBUG_calculateLines = 0;
const int DEBUG_objectsListToStruct = 0;
const int DEBUG_translateObjectToMapFrame = 0;
const int DEBUG_print_foundObjects_msg = 0;

const int DEBUG_main = 0;
const int DEBUG_finish_file_printout = 0;

std::string wheelchair_dump_loc;


struct Objects { //struct for publishing topic
    int id; //get object id from ros msg
    string object_name; //get object name/class
    double object_confidence; //get object confidence

    double point_x; //get transform point x
    double point_y; //get transform point y
    double point_z; //get transform point z

    double quat_x; //get transform rotation quaternion x
    double quat_y; //get transform rotation quaternion y
    double quat_z; //get transform rotation quaternion z
    double quat_w; //get transform rotation quaternion w
};
struct Objects objectsLocationStruct[1000]; //array for storing object data
int totalObjectsLocationStruct = 0; //total objects inside struct

tf::TransformListener *ptrListener; //global pointer for transform listener

ros::Publisher *ptr_publish_objectLocations; //global pointer for publishing topic


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

/*
 * translateObjectToMapFrame()
 * objects_msg is the entire ROS array msg
 * objectID is the array number for the ROS array msg
 * DETframename is the local detection transform for translating to map frame
*/
void translateObjectToMapFrame(const wheelchair_msgs::foundObjects objects_msg, int objectID, std::string DETframename) {
    std::string msg_object_name;
    msg_object_name = objects_msg.object_name[objectID];
    double msg_object_confidence = objects_msg.object_confidence[objectID];
    tf::StampedTransform translation; //initiate translation for transform object
    try {
        ptrListener->waitForTransform("/map", DETframename, ros::Time(0), ros::Duration(3.0)); //wait a few seconds for ROS to respond
        ptrListener->lookupTransform("/map", DETframename, ros::Time(), translation); //lookup translation of object from map frame

        //get global translation of object
        float translation_x = translation.getOrigin().x(); //set translation x to local variable
        float translation_y = translation.getOrigin().y(); //set translation y to local variable
        float translation_z = translation.getOrigin().z(); //set translation z to local variable
        float rotation_x = translation.getRotation().x(); //set rotation x to local variable
        float rotation_y = translation.getRotation().y(); //set rotation y to local variable
        float rotation_z = translation.getRotation().z(); //set rotation z to local variable
        float rotation_w = translation.getRotation().w(); //set rotation w to local variable

        if (DEBUG_translateObjectToMapFrame) {
            printSeparator(0);
            cout << msg_object_name << ", " << msg_object_confidence << endl; //print out object name
            cout << translation_x << ", " << translation_y << ", " << translation_z << ", " << rotation_x << ", " << rotation_y << ", " << rotation_z << ", " << rotation_w << endl;
        }
        objectsLocationStruct[totalObjectsLocationStruct].id = totalObjectsLocationStruct;
        objectsLocationStruct[totalObjectsLocationStruct].object_name = msg_object_name;
        objectsLocationStruct[totalObjectsLocationStruct].object_confidence = msg_object_confidence;

        objectsLocationStruct[totalObjectsLocationStruct].point_x = translation_x;
        objectsLocationStruct[totalObjectsLocationStruct].point_y = translation_y;
        objectsLocationStruct[totalObjectsLocationStruct].point_z = translation_z;

        objectsLocationStruct[totalObjectsLocationStruct].quat_x = rotation_x;
        objectsLocationStruct[totalObjectsLocationStruct].quat_y = rotation_y;
        objectsLocationStruct[totalObjectsLocationStruct].quat_z = rotation_z;
        objectsLocationStruct[totalObjectsLocationStruct].quat_w = rotation_w;
        totalObjectsLocationStruct++; //add 1 to total objects in storage struct - ready for next time
    }
    catch (tf::TransformException ex){
        cout << "Couldn't get translation..." << endl; //catchment function if it can't get a translation from the map
        ROS_ERROR("%s",ex.what()); //print error
        ros::Duration(1.0).sleep();
    }
}

std::string publishLocalDetectionTransform(const wheelchair_msgs::foundObjects objects_msg, int isObject) {
    //broadcast detected objects in frame
    static tf::TransformBroadcaster br; //initialise broadcaster class
    std::string DETframename = "DET:" + objects_msg.object_name[isObject] + std::to_string(isObject); //add frame DET object name
    tf::Transform localTransform;
    //create local transform from zed camera to object
    localTransform.setOrigin( tf::Vector3(objects_msg.point_x[isObject], objects_msg.point_y[isObject], objects_msg.point_z[isObject]) ); //create transform vector
    tf::Quaternion localQuaternion; //initialise quaternion class
    localQuaternion.setRPY(objects_msg.rotation_r[isObject], objects_msg.rotation_p[isObject], objects_msg.rotation_y[isObject]);  //where r p y are fixed
    localTransform.setRotation(localQuaternion); //set quaternion from struct data
    br.sendTransform(tf::StampedTransform(localTransform, ros::Time::now(), "base_footprint", DETframename)); //broadcast transform frame from zed camera link
    //end the temporary frame publishing
    return DETframename;
}

void publishObjectStructMsg() {
    wheelchair_msgs::objectLocations obLoc;
    //publish all objects inside struct
    for (int isObject = 0; isObject < totalObjectsLocationStruct; isObject++) { //iterate through entire struct
        obLoc.id.push_back(objectsLocationStruct[isObject].id);
        obLoc.object_name.push_back(objectsLocationStruct[isObject].object_name);
        obLoc.object_confidence.push_back(objectsLocationStruct[isObject].object_confidence);

        obLoc.point_x.push_back(objectsLocationStruct[isObject].point_x);
        obLoc.point_y.push_back(objectsLocationStruct[isObject].point_y);
        obLoc.point_z.push_back(objectsLocationStruct[isObject].point_z);

        obLoc.quat_x.push_back(objectsLocationStruct[isObject].quat_x);
        obLoc.quat_y.push_back(objectsLocationStruct[isObject].quat_y);
        obLoc.quat_z.push_back(objectsLocationStruct[isObject].quat_z);
        obLoc.quat_w.push_back(objectsLocationStruct[isObject].quat_w);
    }
    obLoc.totalObjects = totalObjectsLocationStruct; //set total objects found in struct
    ptr_publish_objectLocations->publish(obLoc); //publish struct as ros msg array
}

//print out entire objects ros msg from depth_sensing node
void printFoundObjectsMsg(const wheelchair_msgs::foundObjects objects_msg, const int isObject) {
    printSeparator(0);
    cout << "ID: " << objects_msg.id[isObject] << endl;
    cout << "Name: " << objects_msg.object_name[isObject] << endl;
    cout << "Confidence: " << objects_msg.object_confidence[isObject] << endl;

    cout << "Point_x: " << objects_msg.point_x[isObject] << endl;
    cout << "Point_y: " << objects_msg.point_y[isObject] << endl;
    cout << "Point_z: " << objects_msg.point_z[isObject] << endl;

    cout << "Rotation_r: " << objects_msg.rotation_r[isObject] << endl;
    cout << "Rotation_p: " << objects_msg.rotation_p[isObject] << endl;
    cout << "Rotation_y: " << objects_msg.rotation_y[isObject] << endl;
}

void objectsDetectedCallback(const wheelchair_msgs::foundObjects objects_msg) {
    //stuff here on each callback
    //if object isn't detected in room - reduce object influence (instead of deleting?)
    int totalObjects = objects_msg.totalObjects; //get quantity of objects in ROS msg
    for (int isObject = 0; isObject < totalObjects; isObject++) { //iterate through entire ROS msg
        if (DEBUG_print_foundObjects_msg) {
            printFoundObjectsMsg(objects_msg, isObject);
        }
        std::string DETframename = publishLocalDetectionTransform(objects_msg, isObject); //publish DET transform for detected object
        translateObjectToMapFrame(objects_msg, isObject, DETframename);
    }
    publishObjectStructMsg(); //publish ROS msg for publish object locations node
    totalObjectsLocationStruct = 0; //set object locations struct back to 0 once translations have been published
}



int main(int argc, char **argv) {
    //stuff to go here
    ros::init(argc, argv, "object_locations");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("wheelchair_robot/dacop/depth_sensing/detected_objects", 10, objectsDetectedCallback); //callback function when objects are detected from depth_sensing
    ros::Publisher local_publish_objectLocations = n.advertise<wheelchair_msgs::objectLocations>("wheelchair_robot/dacop/object_locations/detected_objects", 1000); //publish to central publishing locations node
    ptr_publish_objectLocations = &local_publish_objectLocations; //point this local pub variable to global status, so the publish function can access it.

    while (ros::ok()) {
        tf::TransformListener listener;
        ptrListener = &listener; //set to global pointer - to access from another function

        ros::Rate rate(10.0);
        
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spin();
        rate.sleep();
    }
    
    //start closing procedure
    cout << "closing ROS node" << endl;
    if (DEBUG_finish_file_printout) {
        printSeparator(0);
        cout << "file output" << endl;
        for (int objectNumber = 0; objectNumber < totalObjectsLocationStruct; objectNumber++) {
            cout << objectsLocationStruct[objectNumber].id << "," << objectsLocationStruct[objectNumber].object_name << "," << objectsLocationStruct[objectNumber].object_confidence << endl;
            cout << objectsLocationStruct[objectNumber].point_x << ", " << objectsLocationStruct[objectNumber].point_y << ", " << objectsLocationStruct[objectNumber].point_z << endl;
            cout << objectsLocationStruct[objectNumber].quat_x << ", " << objectsLocationStruct[objectNumber].quat_y << ", " << objectsLocationStruct[objectNumber].quat_z << ", " << objectsLocationStruct[objectNumber].quat_w << endl;
        }
        printSeparator(0);
    }
    return 0;
}
