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
//#include "std_msgs/String.h"
//#include "std_msgs/Int16.h"
//#include "std_msgs/Float32.h"
//#include "std_msgs/Float64.h"

//experimental
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

/*#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>*/


#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
//#include "tf/transform_datatypes.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include <tf/LinearMath/Quaternion.h>
//#include <cmath>
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
const int DEBUG_objectsListToStruct = 1;
const int DEBUG_print_foundObjects_msg = 0;
const int DEBUG_doesObjectAlreadyExist = 1;
const int DEBUG_main = 1;
const int DEBUG_finish_file_printout = 1;

std::string wheelchair_dump_loc;


struct Objects { //struct for publishing topic
    int id;
    string object_name;
    double point_x;
    double point_y;
    double point_z;

    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
};
struct Objects objectsFileStruct[10000];
int objectsFileTotalLines = 0;
int totalObjectsFileStruct = 0;
double objectTopologyThreshold = 0.5; //this should probably be a bounding box value...

tf::TransformListener *ptrListener;

ros::Publisher *ptr_publish_objectLocations;


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
    std::string objectsDelimiter = ",";
	ifstream FILE_READER(objects_file_loc);
    int objectNumber = 0;
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
    }
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
    try {
        ptrListener->waitForTransform("/map", DETframename, ros::Time(0), ros::Duration(3.0));
        ptrListener->lookupTransform("/map", DETframename, ros::Time(), translation);

        //get global translation of object
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
            cout << translation_x << ", " << translation_y << ", " << translation_z << ", " << rotation_x << ", " << rotation_y << ", " << rotation_z << ", " << rotation_w << endl;
        }
        if (totalObjectsFileStruct == 0) {
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
        int foundObjectMatch = 0;
        for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
            cout << "object no is " << isObject << endl;
            double minPointThreshold_x = objectsFileStruct[isObject].point_x - objectTopologyThreshold; //make minimum x bound
            double maxPointThreshold_x = objectsFileStruct[isObject].point_x + objectTopologyThreshold; //make maximum x bound
            double minPointThreshold_y = objectsFileStruct[isObject].point_y - objectTopologyThreshold; //make minimum y bound
            double maxPointThreshold_y = objectsFileStruct[isObject].point_y + objectTopologyThreshold; //make maximum y bound
            if (DEBUG_doesObjectAlreadyExist) {
                cout << "objectsFileStruct " << objectsFileStruct[isObject].point_x << ", minPointThreshold_x " << minPointThreshold_x << 
                " maxPointThreshold_x, " << maxPointThreshold_x << endl;
                cout << "objectsFileStruct " << objectsFileStruct[isObject].point_y << ", minPointThreshold_y " << minPointThreshold_y << 
                " maxPointThreshold_y, " << maxPointThreshold_y << endl;
            }
            if ( ((translation_x >= minPointThreshold_x) && (translation_x <= maxPointThreshold_x)) && //if it's in x bound
                ((translation_y >= minPointThreshold_y) && (translation_y <= maxPointThreshold_y)) &&
                msg_object_name == objectsFileStruct[isObject].object_name) { //if it's the same object
                    cout << "found same object in this location" << endl;
                    foundObjectMatch = 1;
                
            }
            else {
                //no match found in list, leave at 0
                cout << "no match" << endl;
            }
        }
        if (foundObjectMatch == 1) {
            //found object match, don't do anything
        }
        else if (foundObjectMatch == 0) {
            //found new object, add to struct and iterate the totalObjects
            //add object to last position in struct
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
    }
    catch (tf::TransformException ex){
        cout << "Couldn't get translation..." << endl;
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void publishObjectStruct() {
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

        //start publishing objects struct as a ROS message
        
        obLoc.id.push_back(objectsFileStruct[isObject].id);
        obLoc.object_name.push_back(objectsFileStruct[isObject].object_name);
        obLoc.point_x.push_back(objectsFileStruct[isObject].point_x);
        obLoc.point_y.push_back(objectsFileStruct[isObject].point_y);
        obLoc.point_z.push_back(objectsFileStruct[isObject].point_z);
        obLoc.quat_x.push_back(objectsFileStruct[isObject].quat_x);
        obLoc.quat_y.push_back(objectsFileStruct[isObject].quat_y);
        obLoc.quat_z.push_back(objectsFileStruct[isObject].quat_z);
        obLoc.quat_w.push_back(objectsFileStruct[isObject].quat_w);
    }
    obLoc.totalObjects = totalObjectsFileStruct;
    ptr_publish_objectLocations->publish(obLoc);
}

void printFoundObjectsMsg(const wheelchair_msgs::foundObjects objects_msg, const int isObject) {
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

void objectsDetectedCallback(const wheelchair_msgs::foundObjects objects_msg) {
    //stuff here on each callback
    //if object isn't detected in room - reduce object influence (instead of deleting?)

    int totalObjects = objects_msg.totalObjects;
    for (int isObject = 0; isObject < totalObjects; isObject++) {
        if (DEBUG_print_foundObjects_msg) {
            printFoundObjectsMsg(objects_msg, isObject);
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
        //create local transform from zed camera to object
        localTransform.setOrigin( tf::Vector3(objects_msg.point_x[isObject], objects_msg.point_y[isObject], objects_msg.point_z[isObject]) );
        tf::Quaternion localQuaternion;
        localQuaternion.setRPY(objects_msg.rotation_r[isObject], objects_msg.rotation_p[isObject], objects_msg.rotation_y[isObject]);  //where r p y are fixed
        localTransform.setRotation(localQuaternion);
        br.sendTransform(tf::StampedTransform(localTransform, ros::Time::now(), "zed_left_camera_depth_link", DETframename));
        //end the temporary frame publishing

        //doesObjectAlreadyExist(objects_msg.object_name[isObject], DETframename);
        doesObjectAlreadyExist(objects_msg.object_name[isObject], DETframename);
    }
    publishObjectStruct();
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
        objectsFileTotalLines = calculateLines(objects_file_loc);
        objectsListToStruct(objects_file_loc);
        
        ros::Rate rate(10.0);
        


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
    
    //start closing procedure
    cout << "closing ROS node" << endl;
    objectsStructToList(objects_file_loc);
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