/*
 * publish_object_locations.cpp
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

const int DEBUG_doesWheelchairDumpPkgExist = 0;
const int DEBUG_createFile = 0;
const int DEBUG_calculateLines = 0;
const int DEBUG_objectsListToStruct = 0;
const int DEBUG_publishExistingObjects = 0;
const int DEBUG_doesObjectAlreadyExist = 0;
const int DEBUG_broadcastTransformStruct = 0;
const int DEBUG_objectLocationsCallback = 0;
const int DEBUG_main = 0;

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
struct Objects objectsFileStruct[100000]; //array for storing object data
int totalObjectsFileStruct = 0; //total objects inside struct
double objectTopologyThreshold = 0.5; //this should probably be a bounding box value...

ros::Publisher *ptr_publish_objectLocations; //global pointer for publishing topic
ros::Publisher *ptr_publish_objectUID; //global pointer for publishing topic

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
            objectNumber++; //iterate to next object in list
        }
    }
    totalObjectsFileStruct = objectNumber; //var to add number of objects in struct
}

void publishExistingObjects(const struct Objects existingObjects[1000], int totalExistingObjects) { //publish detected objects with new (static) UIDs
    wheelchair_msgs::objectLocations exisObLoc; //create another objects locations ROS msg
    for (int isExistingObject = 0; isExistingObject < totalExistingObjects; isExistingObject++) { //run through loop of detected objects
        if (DEBUG_publishExistingObjects) {
            cout << existingObjects[isExistingObject].id << ", " << existingObjects[isExistingObject].object_name << endl;
        }
        exisObLoc.id.push_back(existingObjects[isExistingObject].id);
        exisObLoc.object_name.push_back(existingObjects[isExistingObject].object_name);
        exisObLoc.object_confidence.push_back(existingObjects[isExistingObject].object_confidence);

        exisObLoc.point_x.push_back(existingObjects[isExistingObject].point_x);
        exisObLoc.point_y.push_back(existingObjects[isExistingObject].point_y);
        exisObLoc.point_z.push_back(existingObjects[isExistingObject].point_z);

        exisObLoc.quat_x.push_back(existingObjects[isExistingObject].quat_x);
        exisObLoc.quat_y.push_back(existingObjects[isExistingObject].quat_y);
        exisObLoc.quat_z.push_back(existingObjects[isExistingObject].quat_z);
        exisObLoc.quat_w.push_back(existingObjects[isExistingObject].quat_w);
    }
    exisObLoc.totalObjects = totalExistingObjects; //set total objects detected
    ptr_publish_objectUID->publish(exisObLoc); //publish objects detected with new UIDs
}

void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg

    struct Objects existingObjects[1000]; //store UID of new and existing objects for publishing
    int isExistingObject = 0;
    int totalExistingObjects = totalObjectsInMsg; //number of objects in struct

    for (int isMsgObject = 0; isMsgObject < totalObjectsInMsg; isMsgObject++) { 
        //check to see if detected objects in ROS msg are already present in object struct
        std::string msg_object_name = obLoc.object_name[isMsgObject]; //set object name
        double msg_object_confidence = obLoc.object_confidence[isMsgObject]; //set object confidence
        float msg_translation_x = obLoc.point_x[isMsgObject]; //set translation x to local variable
        float msg_translation_y = obLoc.point_y[isMsgObject]; //set translation y to local variable
        float msg_translation_z = obLoc.point_z[isMsgObject]; //set translation z to local variable
        float msg_rotation_x = obLoc.quat_x[isMsgObject]; //set rotation x to local variable
        float msg_rotation_y = obLoc.quat_y[isMsgObject]; //set rotation y to local variable
        float msg_rotation_z = obLoc.quat_z[isMsgObject]; //set rotation z to local variable
        float msg_rotation_w = obLoc.quat_w[isMsgObject]; //set rotation w to local variable
        if (totalObjectsFileStruct == 0) { //can't start for loop if struct is empty - so add some initial data
            //add local variables from above to struct array to store object data referencing map frame
            objectsFileStruct[totalObjectsFileStruct].id = totalObjectsFileStruct; //set first object id to 0
            objectsFileStruct[totalObjectsFileStruct].object_name = msg_object_name;
            objectsFileStruct[totalObjectsFileStruct].object_confidence = msg_object_confidence;

            objectsFileStruct[totalObjectsFileStruct].point_x = msg_translation_x;
            objectsFileStruct[totalObjectsFileStruct].point_y = msg_translation_y;
            objectsFileStruct[totalObjectsFileStruct].point_z = msg_translation_z;

            objectsFileStruct[totalObjectsFileStruct].quat_x = msg_rotation_x;
            objectsFileStruct[totalObjectsFileStruct].quat_y = msg_rotation_y;
            objectsFileStruct[totalObjectsFileStruct].quat_z = msg_rotation_z;
            objectsFileStruct[totalObjectsFileStruct].quat_w = msg_rotation_z;
            totalObjectsFileStruct++; //set object id to 1 - and start looping through objects struct
        }

        int foundObjectMatch = 0; //set found corresponding object to 0 - not found object
        int objectInPosition = 0; //variable to set when object in struct is a match with object in ROS msg
        for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //run through entire struct
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
            } //finish debug statement
            if ( ((msg_translation_x >= minPointThreshold_x) && (msg_translation_x <= maxPointThreshold_x)) && //if there's an object in x bound
                ((msg_translation_y >= minPointThreshold_y) && (msg_translation_y <= maxPointThreshold_y)) && //if there's an object in y bound
                msg_object_name == objectsFileStruct[isObject].object_name) { //if it has classified the same object (name)
                if (DEBUG_doesObjectAlreadyExist) {
                    cout << "found same object in this location" << endl; //print out found object
                }
                foundObjectMatch = 1; //set found object match to 1 - true
                objectInPosition = isObject;
            }
            else {
                //no match found in list, leave at 0
                if (DEBUG_doesObjectAlreadyExist) {
                    cout << "no match" << endl; //print out no match found
                }
            }
        }
        if (foundObjectMatch == 1) { //check to see if the loop above found a match
            //found object match, don't do anything
            //add details to existing objects struct - for getting the UID of new and existing objects
                existingObjects[isExistingObject].id = objectsFileStruct[objectInPosition].id;
                existingObjects[isExistingObject].object_name = objectsFileStruct[objectInPosition].object_name;
                existingObjects[isExistingObject].object_confidence = objectsFileStruct[objectInPosition].object_confidence;

                existingObjects[isExistingObject].point_x = objectsFileStruct[objectInPosition].point_x;
                existingObjects[isExistingObject].point_y = objectsFileStruct[objectInPosition].point_y;
                existingObjects[isExistingObject].point_z = objectsFileStruct[objectInPosition].point_z;

                existingObjects[isExistingObject].quat_x = objectsFileStruct[objectInPosition].quat_x;
                existingObjects[isExistingObject].quat_y = objectsFileStruct[objectInPosition].quat_y;
                existingObjects[isExistingObject].quat_z = objectsFileStruct[objectInPosition].quat_z;
                existingObjects[isExistingObject].quat_w = objectsFileStruct[objectInPosition].quat_w;
                isExistingObject++;
        }
        else if (foundObjectMatch == 0) { //if object is not in the struct
            //found new object, add to struct and iterate the totalObjects
            //add object to last position in struct
                objectsFileStruct[totalObjectsFileStruct].id = totalObjectsFileStruct;
                objectsFileStruct[totalObjectsFileStruct].object_name = msg_object_name;
                objectsFileStruct[totalObjectsFileStruct].object_confidence = msg_object_confidence;

                objectsFileStruct[totalObjectsFileStruct].point_x = msg_translation_x;
                objectsFileStruct[totalObjectsFileStruct].point_y = msg_translation_y;
                objectsFileStruct[totalObjectsFileStruct].point_z = msg_translation_z;

                objectsFileStruct[totalObjectsFileStruct].quat_x = msg_rotation_x;
                objectsFileStruct[totalObjectsFileStruct].quat_y = msg_rotation_y;
                objectsFileStruct[totalObjectsFileStruct].quat_z = msg_rotation_z;
                objectsFileStruct[totalObjectsFileStruct].quat_w = msg_rotation_w;
                totalObjectsFileStruct++; //add 1 to total objects in storage struct - ready for next time

                //add details to existing objects struct - for getting the UID of new and existing objects
                existingObjects[isExistingObject].id = objectsFileStruct[totalObjectsFileStruct].id;
                existingObjects[isExistingObject].object_name = objectsFileStruct[totalObjectsFileStruct].object_name;
                existingObjects[isExistingObject].object_confidence = objectsFileStruct[totalObjectsFileStruct].object_confidence;

                existingObjects[isExistingObject].point_x = objectsFileStruct[totalObjectsFileStruct].point_x;
                existingObjects[isExistingObject].point_y = objectsFileStruct[totalObjectsFileStruct].point_y;
                existingObjects[isExistingObject].point_z = objectsFileStruct[totalObjectsFileStruct].point_z;

                existingObjects[isExistingObject].quat_x = objectsFileStruct[totalObjectsFileStruct].quat_x;
                existingObjects[isExistingObject].quat_y = objectsFileStruct[totalObjectsFileStruct].quat_y;
                existingObjects[isExistingObject].quat_z = objectsFileStruct[totalObjectsFileStruct].quat_z;
                existingObjects[isExistingObject].quat_w = objectsFileStruct[totalObjectsFileStruct].quat_w;
                isExistingObject++;
        }
    }
    publishExistingObjects(existingObjects, totalExistingObjects);
}

void broadcastTransformStruct() {
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
        if (DEBUG_broadcastTransformStruct) {
            cout << "publishing map frame" << endl;
        }
    }
}

void publishObjectStructMsg() {
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
    ros::Subscriber sub = n.subscribe("wheelchair_robot/dacop/object_locations/detected_objects", 10, objectLocationsCallback);
    ros::Publisher local_publish_objectLocations = n.advertise<wheelchair_msgs::objectLocations>("wheelchair_robot/dacop/publish_object_locations/objects", 1000); //publish to central publishing locations node
    ros::Publisher local_publish_objectUID = n.advertise<wheelchair_msgs::objectLocations>("wheelchair_robot/dacop/publish_object_locations/detected_objects", 1000); //publish to central publishing locations node
    ptr_publish_objectLocations = &local_publish_objectLocations; //point this local pub variable to global status, so the publish function can access it.
    ptr_publish_objectUID = &local_publish_objectUID; //point this local pub variable to global status, so the publish function can access it.
    //other subscribers can be added to modify the central objects struct to list
    ros::Rate rate(10.0);

    wheelchair_dump_loc = ros::package::getPath("wheelchair_dump"); //get path for dump directory
    std::string objects_file_loc = wheelchair_dump_loc + "/dump/dacop/objects.dacop"; //set path for dacop file (object info)

    doesWheelchairDumpPkgExist();//check to see if dump package exists
    createFile(objects_file_loc); //create file if it doesn't exist
    objectsListToStruct(objects_file_loc); //add list to struct

    while(ros::ok()) {

        broadcastTransformStruct();
        publishObjectStructMsg();
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