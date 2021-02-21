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
const int DEBUG_print_foundObjects_msg = 0;
const int DEBUG_doesObjectAlreadyExist = 0;
const int DEBUG_main = 0;
const int DEBUG_finish_file_printout = 0;

std::string wheelchair_dump_loc;


struct Objects { //struct for publishing topic
    int id; //get object id from ros msg
    string object_name; //get object name/class
    double object_confidence; //get object confidence
    string room_name; //get room name
    double point_x; //get transform point x
    double point_y; //get transform point y
    double point_z; //get transform point z

    double quat_x; //get transform rotation quaternion x
    double quat_y; //get transform rotation quaternion y
    double quat_z; //get transform rotation quaternion z
    double quat_w; //get transform rotation quaternion w
};
struct Objects objectsFileStruct[10000]; //array for storing object data
int totalObjectsFileStruct = 0; //total objects inside struct
double objectTopologyThreshold = 0.5; //this should probably be a bounding box value...

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

void objectsListToStruct(std::string objects_file_loc) {
    //add list to stuct - test this first before callback
    //contains transforms between map and object
    //id, object_name, point_x, point_y, point_z, quat_x, quat_y, quat_z, quat_w
    std::string objectsDelimiter = ","; //delimiter character is comma
	ifstream FILE_READER(objects_file_loc); //open file
    int objectNumber = 0; //iterate on each object
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
                    objectsFileStruct[objectNumber].id = objectNumber; //set id of object back to 0
                }
                else if (lineSection == 1) {
                    objectsFileStruct[objectNumber].object_name = token; //set object name
                }
                else if (lineSection == 2) {
                    objectsFileStruct[objectNumber].object_confidence = std::stof(token); //set object confidence
                }
                else if (lineSection == 3) {
                    objectsFileStruct[objectNumber].point_x = std::stof(token); //set transform point x
                }
                else if (lineSection == 4) {
                    objectsFileStruct[objectNumber].point_y = std::stof(token); //set transform point y
                }
                else if (lineSection == 5) {
                    objectsFileStruct[objectNumber].point_z = std::stof(token); //set transform point z
                }
                else if (lineSection == 6) {
                    objectsFileStruct[objectNumber].quat_x = std::stof(token); //set rotation to quaternion x
                }
                else if (lineSection == 7) {
                    objectsFileStruct[objectNumber].quat_y = std::stof(token);//set rotation to quaternion y
                }
                else if (lineSection == 8) {
                    objectsFileStruct[objectNumber].quat_z = std::stof(token); //set rotation to quaternion z
                }

                lineSection++;
            }
            //std::cout << line << std::endl;
            objectsFileStruct[objectNumber].quat_w = std::stof(line); //set rotation to quaternion w
            if (DEBUG_objectsListToStruct) { //print off debug lines
                cout << "sections in line " << lineSection << endl;
                cout << objectsFileStruct[objectNumber].id << "," << objectsFileStruct[objectNumber].object_name << ", " << objectsFileStruct[objectNumber].object_confidence << endl;
                cout << objectsFileStruct[objectNumber].point_x << ", " << objectsFileStruct[objectNumber].point_y << ", " << objectsFileStruct[objectNumber].point_z << endl;
                cout << objectsFileStruct[objectNumber].quat_x << ", " << objectsFileStruct[objectNumber].quat_y << ", " << objectsFileStruct[objectNumber].quat_z << ", " << objectsFileStruct[objectNumber].quat_w << endl;
                printSeparator(0);
            }
        }
        objectNumber++; //iterate to next object in list
    }
    totalObjectsFileStruct = objectNumber; //var to add number of objects in struct
}

void doesObjectAlreadyExist(std::string msg_object_name, std::string DETframename) {
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

        if (DEBUG_doesObjectAlreadyExist) {
            printSeparator(0);
            cout << msg_object_name << endl; //print out object name
            cout << translation_x << ", " << translation_y << ", " << translation_z << ", " << rotation_x << ", " << rotation_y << ", " << rotation_z << ", " << rotation_w << endl;
        }
        if (totalObjectsFileStruct == 0) {
            //add local variables from above to struct array to store object data referencing map frame
            objectsFileStruct[totalObjectsFileStruct].id = totalObjectsFileStruct;
            objectsFileStruct[totalObjectsFileStruct].object_name = msg_object_name;
            objectsFileStruct[totalObjectsFileStruct].point_x = translation_x;
            objectsFileStruct[totalObjectsFileStruct].point_y = translation_y;
            objectsFileStruct[totalObjectsFileStruct].point_z = translation_z;
            objectsFileStruct[totalObjectsFileStruct].quat_x = rotation_x;
            objectsFileStruct[totalObjectsFileStruct].quat_y = rotation_y;
            objectsFileStruct[totalObjectsFileStruct].quat_z = rotation_z;
            objectsFileStruct[totalObjectsFileStruct].quat_w = rotation_w;
            totalObjectsFileStruct++;
        }
        int foundObjectMatch = 0; //set found corresponding object to 0 - not found object
        for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
            if (DEBUG_doesObjectAlreadyExist) {
                cout << "object no is " << isObject << endl; //print out object number
            }
            //set calculations to create a distance threshold - if object is in this box, it's probably the same object
            double minPointThreshold_x = objectsFileStruct[isObject].point_x - objectTopologyThreshold; //make minimum x bound
            double maxPointThreshold_x = objectsFileStruct[isObject].point_x + objectTopologyThreshold; //make maximum x bound
            double minPointThreshold_y = objectsFileStruct[isObject].point_y - objectTopologyThreshold; //make minimum y bound
            double maxPointThreshold_y = objectsFileStruct[isObject].point_y + objectTopologyThreshold; //make maximum y bound
            if (DEBUG_doesObjectAlreadyExist) { //print out current object and global box calculations
                cout << "objectsFileStruct " << objectsFileStruct[isObject].point_x << ", minPointThreshold_x " << minPointThreshold_x << 
                " maxPointThreshold_x, " << maxPointThreshold_x << endl;
                cout << "objectsFileStruct " << objectsFileStruct[isObject].point_y << ", minPointThreshold_y " << minPointThreshold_y << 
                " maxPointThreshold_y, " << maxPointThreshold_y << endl;
            }
            if ( ((translation_x >= minPointThreshold_x) && (translation_x <= maxPointThreshold_x)) && //if there's an object in x bound
                ((translation_y >= minPointThreshold_y) && (translation_y <= maxPointThreshold_y)) && //if there's an object in y bound
                msg_object_name == objectsFileStruct[isObject].object_name) { //if it has classified the same object (name)
                    if (DEBUG_doesObjectAlreadyExist) {
                        cout << "found same object in this location" << endl; //print out found object
                    }
                    foundObjectMatch = 1; //set found object match to 1 - true
                
            }
            else {
                //no match found in list, leave at 0
                if (DEBUG_doesObjectAlreadyExist) {
                    cout << "no match" << endl; //print out no match found
                }
            }
        }
        if (foundObjectMatch == 1) {
            //found object match, don't do anything
        }
        else if (foundObjectMatch == 0) { //if object is not in the list
            //found new object, add to struct and iterate the totalObjects
            //add object to last position in struct
                objectsFileStruct[totalObjectsFileStruct].id = totalObjectsFileStruct;
                objectsFileStruct[totalObjectsFileStruct].object_name = msg_object_name;
                //add confidence here - double check how the object is transferred here
                objectsFileStruct[totalObjectsFileStruct].point_x = translation_x;
                objectsFileStruct[totalObjectsFileStruct].point_y = translation_y;
                objectsFileStruct[totalObjectsFileStruct].point_z = translation_z;
                objectsFileStruct[totalObjectsFileStruct].quat_x = rotation_x;
                objectsFileStruct[totalObjectsFileStruct].quat_y = rotation_y;
                objectsFileStruct[totalObjectsFileStruct].quat_z = rotation_z;
                objectsFileStruct[totalObjectsFileStruct].quat_w = rotation_w;
                totalObjectsFileStruct++;
        }
    }
    catch (tf::TransformException ex){
        cout << "Couldn't get translation..." << endl; //catchment function if it can't get a translation from the map
        ROS_ERROR("%s",ex.what()); //print error
        ros::Duration(1.0).sleep();
    }
}

void publishObjectStruct() {
    wheelchair_msgs::objectLocations obLoc;
    //publish all objects inside struct
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //iterate through entire struct
        obLoc.id.push_back(objectsFileStruct[isObject].id);
        obLoc.object_name.push_back(objectsFileStruct[isObject].object_name);
        obLoc.object_confidence.push_back(objectsFileStruct[isObject].object_confidence);
        obLoc.point_x.push_back(objectsFileStruct[isObject].point_x);
        obLoc.point_y.push_back(objectsFileStruct[isObject].point_y);
        obLoc.point_z.push_back(objectsFileStruct[isObject].point_z);
        obLoc.quat_x.push_back(objectsFileStruct[isObject].quat_x);
        obLoc.quat_y.push_back(objectsFileStruct[isObject].quat_y);
        obLoc.quat_z.push_back(objectsFileStruct[isObject].quat_z);
        obLoc.quat_w.push_back(objectsFileStruct[isObject].quat_w);
    }
    obLoc.totalObjects = totalObjectsFileStruct; //set total objects found in struct
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

    int totalObjects = objects_msg.totalObjects;
    for (int isObject = 0; isObject < totalObjects; isObject++) {
        if (DEBUG_print_foundObjects_msg) {
            printFoundObjectsMsg(objects_msg, isObject);
        }

        //broadcast detected objects in frame
        static tf::TransformBroadcaster br; //initialise broadcaster class
        std::string DETframename = "DET:" + objects_msg.object_name[isObject] + std::to_string(isObject); //add frame DET object name
        tf::Transform localTransform;
        //create local transform from zed camera to object
        localTransform.setOrigin( tf::Vector3(objects_msg.point_x[isObject], objects_msg.point_y[isObject], objects_msg.point_z[isObject]) ); //create transform vector
        tf::Quaternion localQuaternion; //initialise quaternion class
        localQuaternion.setRPY(objects_msg.rotation_r[isObject], objects_msg.rotation_p[isObject], objects_msg.rotation_y[isObject]);  //where r p y are fixed
        localTransform.setRotation(localQuaternion); //set quaternion from struct data
        br.sendTransform(tf::StampedTransform(localTransform, ros::Time::now(), "zed_left_camera_depth_link", DETframename)); //broadcast transform frame from zed camera link
        //end the temporary frame publishing

        //doesObjectAlreadyExist(objects_msg.object_name[isObject], DETframename);
        doesObjectAlreadyExist(objects_msg.object_name[isObject], DETframename); //does this object already exist, if not, publish it
    }
    publishObjectStruct(); //publish ROS msg for publish object locations node
}



int main(int argc, char **argv) {
    //stuff to go here
    ros::init(argc, argv, "object_locations");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("wheelchair_robot/object_depth/detected_objects", 10, objectsDetectedCallback);
    ros::Publisher local_publish_objectLocations = n.advertise<wheelchair_msgs::objectLocations>("wheelchair_robot/object_locations/objects", 1000);
    ptr_publish_objectLocations = &local_publish_objectLocations; //point this local pub variable to global status, so the publish function can access it.

    wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
    std::string objects_file_loc = wheelchair_dump_loc + "/dump/dacop/objects.dacop";
    while (ros::ok()) {
        tf::TransformListener listener;
        ptrListener = &listener;

        doesWheelchairDumpPkgExist();
        int objectsListExists = createFile(objects_file_loc); //create room list
        objectsListToStruct(objects_file_loc);
        
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
        for (int objectNumber = 0; objectNumber < totalObjectsFileStruct; objectNumber++) {
            cout << objectsFileStruct[objectNumber].id << "," << objectsFileStruct[objectNumber].object_name << endl;
            cout << objectsFileStruct[objectNumber].point_x << ", " << objectsFileStruct[objectNumber].point_y << ", " << objectsFileStruct[objectNumber].point_z << endl;
            cout << objectsFileStruct[objectNumber].quat_x << ", " << objectsFileStruct[objectNumber].quat_y << ", " << objectsFileStruct[objectNumber].quat_z << ", " << objectsFileStruct[objectNumber].quat_w << endl;
        }
        printSeparator(0);
    }
    return 0;
}
