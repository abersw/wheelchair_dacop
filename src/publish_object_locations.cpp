/*
 * publish_object_locations.cpp
 * wheelchair_dacop
 * version: 0.2.0 Majestic Maidenhair
 * Status: Alpha
*/


#include "tof_tool/tof_tool_box.h"

#include <std_srvs/Empty.h>
#include "wheelchair_msgs/foundObjects.h"
#include "wheelchair_msgs/objectLocations.h"

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

using namespace std;

const int DEBUG_forwardCameraTimestamp = 0;
const int DEBUG_objectsListToStruct = 0;
const int DEBUG_publishDetectedObjects = 0;
const int DEBUG_doesObjectAlreadyExist = 0;
const int DEBUG_printObjectLocation = 0;
const int DEBUG_broadcastTransformStruct = 0;
const int DEBUG_objectLocationsCallback = 0;
const int DEBUG_main = 0;

ros::NodeHandle *ptr_n;
TofToolBox *tofToolBox;

//struct for publishing topic r.e. object locations in world
struct Objects {
    int id; //get object id from ros msg
    string object_name; //get object name/class
    float object_confidence; //get object confidence

    float point_x; //get transform point x
    float point_y; //get transform point y
    float point_z; //get transform point z

    float quat_x; //get transform rotation quaternion x
    float quat_y; //get transform rotation quaternion y
    float quat_z; //get transform rotation quaternion z
    float quat_w; //get transform rotation quaternion w
};
struct Objects objectsFileStruct[100000]; //array for storing all object data in world
int totalObjectsFileStruct = 0; //total objects inside objectsFileStruct struct
double objectBoundingBox = 0.5; //this should probably be a bounding box value...

ros::Publisher *ptr_publish_objectLocations; //global pointer for publishing objectLocations topic
ros::Publisher *ptr_publish_objectUID; //global pointer for publishing topic

std::string wheelchair_dump_loc;

ros::Time camera_timestamp;
double camera_timestamp_sec;




/**
 * assign source camera timestamp to global variable for publishing to object locations
 *
 * @param 'obLoc' is a ROS message of type wheelchair_msgs::objectLocations
 */
void forwardCameraTimestamp(const wheelchair_msgs::objectLocations obLoc) {
    camera_timestamp = obLoc.camera_timestamp;
    camera_timestamp_sec = camera_timestamp.toSec();
    if (DEBUG_forwardCameraTimestamp) {
        cout.precision(17);
        cout << "camera timestamp " << fixed << camera_timestamp << endl;
    }
}

/**
 * Function reads in object list from wheelchair dump and assigns data to struct 'objectsFileStruct'
 *
 * @param 'objects_file_loc' is a string of the path to objects list
 */
void objectsListToStruct(std::string objects_file_loc) {
    //add list to stuct - test this first before callback
    //contains transforms between map and object
    //id, object_name, object_confidence, point_x, point_y, point_z, quat_x, quat_y, quat_z, quat_w
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
                    objectsFileStruct[objectNumber].id = std::stoi(token); //set id of object from existing id
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
                cout << objectsFileStruct[objectNumber].id << "," <<
                        objectsFileStruct[objectNumber].object_name << ", " <<
                        objectsFileStruct[objectNumber].object_confidence << endl;
                cout << objectsFileStruct[objectNumber].point_x << ", " <<
                        objectsFileStruct[objectNumber].point_y << ", " <<
                        objectsFileStruct[objectNumber].point_z << endl;
                cout << objectsFileStruct[objectNumber].quat_x << ", " <<
                        objectsFileStruct[objectNumber].quat_y << ", " <<
                        objectsFileStruct[objectNumber].quat_z << ", " <<
                        objectsFileStruct[objectNumber].quat_w << endl;
                tofToolBox->printSeparator(0);
            }
            objectNumber++; //iterate to next object in list
        }
    }
    totalObjectsFileStruct = objectNumber; //var to add number of objects in struct
}

/**
 * Function prints incoming message of detected objects in frame
 *
 * @param 'obLoc' is ros msg of type wheelchair_msgs::objectLocations - publishing the location data for objects
 * @param 'objectNo' is the position of the object element to print out from the array
 */
void printObjectLocation(const wheelchair_msgs:: objectLocations obLoc, int objectNo) {
    cout << "id: " << obLoc.id[objectNo] << endl <<
    "object name: " << obLoc.object_name[objectNo] << endl <<
    "object confidence: " << obLoc.object_confidence[objectNo] << endl <<

    "x: " << obLoc.point_x[objectNo] << endl <<
    "y: " << obLoc.point_y[objectNo] << endl <<
    "z: " << obLoc.point_z[objectNo] << endl <<

    "x: " << obLoc.quat_x[objectNo] << endl <<
    "y: " << obLoc.quat_y[objectNo] << endl <<
    "z: " << obLoc.quat_z[objectNo] << endl <<
    "w: " << obLoc.quat_w[objectNo] << endl <<

    "total: " << obLoc.totalObjects << endl;
}

/**
 * Function publishes an array of objects detected in the current frame
 *
 * @param 'detectedObjects' is an array of all objects detected in the current frame
 * @param 'totalDetectedObjects' is the total number of detected objects in the current frame
 */
void publishDetectedObjects(const struct Objects detectedObjects[1000], int totalDetectedObjects) { //publish detected objects with new (static) UIDs
    if (DEBUG_publishDetectedObjects) {
        tofToolBox->printSeparator(0);
    }
    wheelchair_msgs::objectLocations exisObLoc; //create another objects locations ROS msg
    exisObLoc.header.stamp = camera_timestamp;
    for (int isExistingObject = 0; isExistingObject < totalDetectedObjects; isExistingObject++) { //run through loop of detected objects
        if (DEBUG_publishDetectedObjects) {
            cout << detectedObjects[isExistingObject].id << ", " << detectedObjects[isExistingObject].object_name << endl;
        }
        exisObLoc.id.push_back(detectedObjects[isExistingObject].id);
        exisObLoc.object_name.push_back(detectedObjects[isExistingObject].object_name);
        exisObLoc.object_confidence.push_back(detectedObjects[isExistingObject].object_confidence);

        exisObLoc.point_x.push_back(detectedObjects[isExistingObject].point_x);
        exisObLoc.point_y.push_back(detectedObjects[isExistingObject].point_y);
        exisObLoc.point_z.push_back(detectedObjects[isExistingObject].point_z);

        exisObLoc.quat_x.push_back(detectedObjects[isExistingObject].quat_x);
        exisObLoc.quat_y.push_back(detectedObjects[isExistingObject].quat_y);
        exisObLoc.quat_z.push_back(detectedObjects[isExistingObject].quat_z);
        exisObLoc.quat_w.push_back(detectedObjects[isExistingObject].quat_w);
    }
    exisObLoc.totalObjects = totalDetectedObjects; //set total objects detected
    ptr_publish_objectUID->publish(exisObLoc); //publish objects detected with new UIDs
}

/**
 * Function creates and broadcasts a transform (tf) for each object in the objectsFileStruct struct
 *
 */
void broadcastTransformStruct() {
    //publish all objects inside struct
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //iterate through entire struct
        int nanDetected = 0;
        int objectID = objectsFileStruct[isObject].id;
        std::string objectName = objectsFileStruct[isObject].object_name;

        static tf::TransformBroadcaster br;
        std::string OBframename = std::to_string(objectID) + objectName;

        //turn msg to pose
        tf::Transform mapTransform;
        //create map transform from map to object frame
        mapTransform.setOrigin(
                tf::Vector3(
                        objectsFileStruct[isObject].point_x,
                        objectsFileStruct[isObject].point_y,
                        objectsFileStruct[isObject].point_z) );
        tf::Quaternion mapQuaternion(
                objectsFileStruct[isObject].quat_x,
                objectsFileStruct[isObject].quat_y,
                objectsFileStruct[isObject].quat_z,
                objectsFileStruct[isObject].quat_w);
        mapTransform.setRotation(mapQuaternion);
        int foundNaN = tofToolBox->validateTransform(mapTransform);
        if (foundNaN) {
            //skip object
            nanDetected = 1;
            cout << "NaN detected when transforming to map frame" << endl;
        }
        else {
            br.sendTransform(
                    tf::StampedTransform(
                            mapTransform,
                            camera_timestamp,
                            "map",
                            OBframename));
            //end the map frame object publishing
            if (DEBUG_broadcastTransformStruct) {
                cout << "publishing map frame" << endl;
                cout << mapTransform.getOrigin().x() << ", " <<
                        mapTransform.getOrigin().y() << ", " <<
                        mapTransform.getOrigin().z() << endl;
            }
        }
    }
}

/**
 * Function publishes the entirety of the objectsFileStruct as a ros msg type wheelchair_msgs::objectLocations
 *
 */
void publishObjectStructMsg() {
    wheelchair_msgs::objectLocations obLoc;
    obLoc.header.stamp = camera_timestamp;
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

/**
 * Function saves contents of objectsFileStruct struct to a list
 *
 * @param 'objects_file_loc' is the file path for saving the objects list
 */
void objectsStructToList(std::string objects_file_loc) {
    //add struct to list file here
    ofstream FILE_WRITER;
	FILE_WRITER.open(objects_file_loc);
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
        FILE_WRITER <<
            objectsFileStruct[isObject].id << "," <<
            objectsFileStruct[isObject].object_name << "," <<
            objectsFileStruct[isObject].object_confidence << "," <<

            objectsFileStruct[isObject].point_x << "," <<
            objectsFileStruct[isObject].point_y << "," <<
            objectsFileStruct[isObject].point_z << "," <<

            objectsFileStruct[isObject].quat_x << "," <<
            objectsFileStruct[isObject].quat_y << "," <<
            objectsFileStruct[isObject].quat_z << "," <<
            objectsFileStruct[isObject].quat_w << "\n";
    }
    FILE_WRITER.close();
    cout << "finished saving function" << endl;
}

/**
 * Main callback function for the node
 * Receives the object location data and compares it to an existing struct of objects to see if it already exists
 * Adds object to struct if it doesn't exist / gets the ID of the object if it already exists
 * Publishes detected objects in frame with new/existing object ID
 *
 * @param 'obLoc' is ros msg of type wheelchair_msgs::objectLocations - publishing the location data for objects
 */
void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    if (DEBUG_objectLocationsCallback) {
        tofToolBox->printSeparator(0);
        cout << "starting new detection ROS array" << endl;
    }
    forwardCameraTimestamp(obLoc); //get camera timestamp via message
    int totalDetectedObjects = obLoc.totalObjects; //get total objects in ROS msg array

    struct Objects detectedObjects[1000]; //struct array for returning objects with correct UID
    int objectID = 0;

    for (int detectedObject = 0; detectedObject < totalDetectedObjects; detectedObject++) { //run through detected objects array
        std::string msg_object_name = obLoc.object_name[detectedObject]; //get detected object name
        double msg_object_confidence = obLoc.object_confidence[detectedObject]; //get detected object confidence

        float msg_translation_x = obLoc.point_x[detectedObject]; //set translation x to local variable
        float msg_translation_y = obLoc.point_y[detectedObject]; //set translation y to local variable
        float msg_translation_z = obLoc.point_z[detectedObject]; //set translation z to local variable

        float msg_rotation_x = obLoc.quat_x[detectedObject]; //set rotation x to local variable
        float msg_rotation_y = obLoc.quat_y[detectedObject]; //set rotation y to local variable
        float msg_rotation_z = obLoc.quat_z[detectedObject]; //set rotation z to local variable
        float msg_rotation_w = obLoc.quat_w[detectedObject]; //set rotation w to local variable
        if (DEBUG_printObjectLocation) {
            printObjectLocation(obLoc, detectedObject);
        }

        int objectAlreadyInStruct = 0; //set found corresponding object to 0 - not found object
        int objectArrayPos = 0; //variable to set when object in struct is a match with object in ROS msg

        for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
            if (DEBUG_doesObjectAlreadyExist) {
                cout << "main struct id is " << isObject << endl;
            }
            //set calculations to create a distance threshold - if object is in this box, it's probably the same object
            float minPointThreshold_x = objectsFileStruct[isObject].point_x - objectBoundingBox; //make minimum x bound
            float maxPointThreshold_x = objectsFileStruct[isObject].point_x + objectBoundingBox; //make maximum x bound
            float minPointThreshold_y = objectsFileStruct[isObject].point_y - objectBoundingBox; //make minimum y bound
            float maxPointThreshold_y = objectsFileStruct[isObject].point_y + objectBoundingBox; //make maximum y bound
            float minPointThreshold_z = objectsFileStruct[isObject].point_z - objectBoundingBox; //make minimum z bound
            float maxPointThreshold_z = objectsFileStruct[isObject].point_z + objectBoundingBox; //make maximum z bound

            if ( ((msg_translation_x >= minPointThreshold_x) && (msg_translation_x <= maxPointThreshold_x)) && //if there's an object in x bound
                ((msg_translation_y >= minPointThreshold_y) && (msg_translation_y <= maxPointThreshold_y)) && //if there's an object in y bound
                ((msg_translation_z >= minPointThreshold_z) && (msg_translation_z <= maxPointThreshold_z)) && //if there's an object in y bound
                msg_object_name == objectsFileStruct[isObject].object_name) { //if it has classified the same object (name)
                if (DEBUG_doesObjectAlreadyExist) {
                    cout << "found same object in this location" << endl; //print out found object
                }
                objectAlreadyInStruct = 1; //set found object match to 1 - true
                objectArrayPos = isObject;
            }
            else {
                //no match found in list, leave at 0
                if (DEBUG_doesObjectAlreadyExist) {
                    cout << "no match" << endl; //print out no match found
                }
            }
        }
        if (objectAlreadyInStruct == 1) {
            //don't do anything, but send matched object details to detected_objects struct array
            detectedObjects[objectID].id = objectsFileStruct[objectArrayPos].id;
            detectedObjects[objectID].object_name = objectsFileStruct[objectArrayPos].object_name;
            detectedObjects[objectID].object_confidence = objectsFileStruct[objectArrayPos].object_confidence;

            detectedObjects[objectID].point_x = objectsFileStruct[objectArrayPos].point_x;
            detectedObjects[objectID].point_y = objectsFileStruct[objectArrayPos].point_y;
            detectedObjects[objectID].point_z = objectsFileStruct[objectArrayPos].point_z;

            detectedObjects[objectID].quat_x = objectsFileStruct[objectArrayPos].quat_x;
            detectedObjects[objectID].quat_y = objectsFileStruct[objectArrayPos].quat_y;
            detectedObjects[objectID].quat_z = objectsFileStruct[objectArrayPos].quat_z;
            detectedObjects[objectID].quat_w = objectsFileStruct[objectArrayPos].quat_w;
            objectID++;
        }
        else {
            //add object main storage struct array
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

            //add object to detected objects struct array
            detectedObjects[objectID].id = objectsFileStruct[totalObjectsFileStruct].id;
            detectedObjects[objectID].object_name = objectsFileStruct[totalObjectsFileStruct].object_name;
            detectedObjects[objectID].object_confidence = objectsFileStruct[totalObjectsFileStruct].object_confidence;

            detectedObjects[objectID].point_x = objectsFileStruct[totalObjectsFileStruct].point_x;
            detectedObjects[objectID].point_y = objectsFileStruct[totalObjectsFileStruct].point_y;
            detectedObjects[objectID].point_z = objectsFileStruct[totalObjectsFileStruct].point_z;

            detectedObjects[objectID].quat_x = objectsFileStruct[totalObjectsFileStruct].quat_x;
            detectedObjects[objectID].quat_y = objectsFileStruct[totalObjectsFileStruct].quat_y;
            detectedObjects[objectID].quat_z = objectsFileStruct[totalObjectsFileStruct].quat_z;
            detectedObjects[objectID].quat_w = objectsFileStruct[totalObjectsFileStruct].quat_w;


            totalObjectsFileStruct++; //iterate after assigning the detectedObjects array 
            objectID++; //iterate to next object in detectedObjects
        }
    }
    publishObjectStructMsg();
    publishDetectedObjects(detectedObjects, totalDetectedObjects); //publish detected objects
}


/**
 * Main initialiser function for node, sets subscribers and publishers, reads in lists of existing objects
 *
 * @param 'obLoc' is ros msg of type wheelchair_msgs::objectLocations - storing the location data for objects
 * @param 'objectNo' is the position of the object element to print out from the array
 */
int main(int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;
    wheelchair_dump_loc = tofToolBox->doesPkgExist("wheelchair_dump");//check to see if dump package exists
    std::string objects_file_loc = wheelchair_dump_loc + "/dump/dacop/objects.dacop"; //set path for dacop file (object info)
    tofToolBox->createFile(objects_file_loc); //create file if it doesn't exist
    objectsListToStruct(objects_file_loc); //add list to struct

    ros::init(argc, argv, "publish_object_locations");

    ros::NodeHandle n;
    ptr_n = &n;
    //called every time an object is detected
    ros::Subscriber sub = n.subscribe("wheelchair_robot/dacop/object_locations/detected_objects", 1000, objectLocationsCallback);
    //publish to central publishing locations node
    ros::Publisher local_publish_objectLocations = n.advertise<wheelchair_msgs::objectLocations>("wheelchair_robot/dacop/publish_object_locations/objects", 1000);
    //publish to central publishing locations node
    ros::Publisher local_publish_objectUID = n.advertise<wheelchair_msgs::objectLocations>("wheelchair_robot/dacop/publish_object_locations/detected_objects", 1000);
    ptr_publish_objectLocations = &local_publish_objectLocations; //point this local pub variable to global status, so the publish function can access it.
    ptr_publish_objectUID = &local_publish_objectUID; //point this local pub variable to global status, so the publish function can access it.
    //other subscribers can be added to modify the central objects struct to list
    ros::Rate rate(50.0);


    while(ros::ok()) {
        //tofToolBox->sayHello(); //test function for tof toolbox
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