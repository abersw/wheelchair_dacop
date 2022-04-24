/*
 * assign_room_to_object.cpp
 * wheelchair_dacop
 * version: 1.0.0 Majestic Maidenhair
 * Status: Gamma
*/

#include "tof_tool/tof_tool_box.h"

#include "std_msgs/String.h" //ROS msg type string

#include "wheelchair_msgs/objectLocations.h"
#include "wheelchair_msgs/roomLocations.h"
#include "wheelchair_msgs/roomToObjects.h"

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <thread>
#include <ros/callback_queue.h>
#include <chrono>

using namespace std;

const int DEBUG_roomListToStruct = 0;
const int DEBUG_roomsDacopToStruct = 0;
const int DEBUG_objectLocationsCallback = 0;
const int DEBUG_detectedObjectsCallback = 0;
const int DEBUG_foundMatchingID = 0;
const int DEBUG_roomNameCallback = 0;
const int DEBUG_broadcastRoomFrame = 0;
const int DEBUG_publishRoomsDacop = 0;
const int DEBUG_saveRoomsList = 1;
const int DEBUG_saveRoomsDacop = 1;
const int DEBUG_main = 0;

TofToolBox *tofToolBox;

struct Rooms {
    int room_id;
    string room_name;

    float point_x; //get transform point x
    float point_y; //get transform point y
    float point_z; //get transform point z

    float quat_x; //get transform rotation quaternion x
    float quat_y; //get transform rotation quaternion y
    float quat_z; //get transform rotation quaternion z
    float quat_w; //get transform rotation quaternion w
};
int totalRoomsFileStruct = 0;
//id,name,pointx,pointy,pointz,quatx,quaty,quatz,quatw
struct Rooms roomsFileStruct[1000];

struct Objects { //struct for publishing topic
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
struct Objects objectsFileStruct[100000]; //array for storing object data
int totalObjectsFileStruct = 0; //total objects inside struct

struct Links {
    int object_id;
    string object_name;

    int room_id;
    string room_name;
};
int totalLinkFileStruct = 0;
struct Links linkFileStruct[100000]; //array for storing all object and room data

std::string userRoomName;
int currentRoomID;
std::string currentRoomName;

ros::Publisher *ptr_publish_roomsDacop; //global pointer for publishing topic
ros::Publisher *ptr_publish_rooms; //global pointer for publishing rooms
tf::TransformListener *ptrListener; //global pointer for transform listener

//list of file locations
std::string wheelchair_dump_loc;
std::string dump_dacop_loc = "/dump/dacop/";
std::string rooms_list_name = "rooms.list"; //file with list of rooms
std::string rooms_list_loc; //full path for rooms list
std::string rooms_dacop_name = "rooms.dacop"; //file with objects associated with rooms
std::string rooms_dacop_loc; //full path for rooms dacop file


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
        cout << fileName << " file is empty" << endl;
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
                if (lineSection == 0) { //if first delimiter
                    roomsFileStruct[roomNumber].room_id = std::stoi(token); //convert room id string to int
                }
                else if (lineSection == 1) {
                    roomsFileStruct[roomNumber].room_name = token;
                }
                else if (lineSection == 2) {
                    roomsFileStruct[roomNumber].point_x = std::stof(token); //set transform point x
                }
                else if (lineSection == 3) {
                    roomsFileStruct[roomNumber].point_y = std::stof(token); //set transform point y
                }
                else if (lineSection == 4) {
                    roomsFileStruct[roomNumber].point_z = std::stof(token); //set transform point z
                }
                else if (lineSection == 5) {
                    roomsFileStruct[roomNumber].quat_x = std::stof(token); //set rotation to quaternion x
                }
                else if (lineSection == 6) {
                    roomsFileStruct[roomNumber].quat_y = std::stof(token);//set rotation to quaternion y
                }
                else if (lineSection == 7) {
                    roomsFileStruct[roomNumber].quat_z = std::stof(token); //set rotation to quaternion z
                }
                lineSection++; //move to next delimiter
            }
            roomsFileStruct[roomNumber].quat_w = std::stof(line); //set end of line to quat w

            if (DEBUG_roomListToStruct) {
                cout << 
                roomsFileStruct[roomNumber].room_id << "," << 
                roomsFileStruct[roomNumber].room_name << "," <<

                roomsFileStruct[roomNumber].point_x << "," <<
                roomsFileStruct[roomNumber].point_y << "," <<
                roomsFileStruct[roomNumber].point_z << "," <<

                roomsFileStruct[roomNumber].quat_x << "," <<
                roomsFileStruct[roomNumber].quat_y << "," <<
                roomsFileStruct[roomNumber].quat_z << "," <<
                roomsFileStruct[roomNumber].quat_w << endl;
            }
            roomNumber++; //go to next room line
        }
    }
    totalRoomsFileStruct = roomNumber; //set total number of rooms in struct to the lines(rooms) counted
}

/**
 * Take room dacop file and add it to struct array for processing later 
 *
 * @param parameter fileName is the path of the room.dacop file
 */
void roomsDacopToStruct(std::string fileName) {
    if (DEBUG_roomsDacopToStruct) {
        cout << "DEBUG_roomsDacopToStruct" << endl;
    }
    //object_id, object_name, room_id, room_name
    std::string objectsDelimiter = ","; //delimiter character is a comma
    ifstream FILE_READER(fileName); //open file
    int objectNumber = 0; //iterate on each object
    if (FILE_READER.peek() == std::ifstream::traits_type::eof()) {
        //don't do anything if next character in file is eof
        cout << fileName << " file is empty" << endl;
    }
    else {
        std::string line;
        while (getline(FILE_READER, line)) { //go through line by line
            int lineSection = 0; //var for iterating through serialised line
            int pos = 0; //position of delimiter
            std::string token;
            while ((pos = line.find(objectsDelimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                line.erase(0, pos + objectsDelimiter.length());
                //deserialise the line section below:
                if (lineSection == 0) { //if first delimiter
                    linkFileStruct[objectNumber].object_id = std::stoi(token); //convert object id string to int
                }
                else if (lineSection == 1) { //if second delimiter
                    linkFileStruct[objectNumber].object_name = token; //get object name
                }
                else if (lineSection == 2) { //if third delimiter
                    linkFileStruct[objectNumber].room_id = std::stoi(token); //convert room id string to int
                }
                lineSection++; //go to next delimiter
            }
            linkFileStruct[objectNumber].room_name = line; //get end of line - room name
            if (DEBUG_roomsDacopToStruct) {
                cout << 
                linkFileStruct[objectNumber].object_id << "," <<
                linkFileStruct[objectNumber].object_name << "," <<
                linkFileStruct[objectNumber].room_id << "," <<
                linkFileStruct[objectNumber].room_name << endl;
            }
            objectNumber++; //go to next object line
        }
    }
    totalLinkFileStruct = objectNumber; //set total number of objects in struct to the lines(obstacles) counted
}

/**
 * Main callback function triggered by msg of all objects
 *
 * @param parameter 'obLoc' is the full list of objects from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void objectLocationsCallback(const wheelchair_msgs::objectLocations::ConstPtr& obLoc) {
    int totalObjectsInMsg = obLoc->totalObjects; //total detected objects in ROS msg
    totalObjectsFileStruct = totalObjectsInMsg; //set message total objects to total objects in file struct
    totalLinkFileStruct = totalObjectsInMsg;
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //iterate through entire msg topic array
        objectsFileStruct[isObject].id = obLoc->id[isObject]; //assign object id to struct
        objectsFileStruct[isObject].object_name = obLoc->object_name[isObject]; //assign object name to struct
        objectsFileStruct[isObject].object_confidence = obLoc->object_confidence[isObject]; //assign object confidence to struct

        objectsFileStruct[isObject].point_x = obLoc->point_x[isObject]; //assign object vector point x to struct
        objectsFileStruct[isObject].point_y = obLoc->point_y[isObject]; //assign object vector point y to struct
        objectsFileStruct[isObject].point_z = obLoc->point_z[isObject]; //assign object vector point z to struct

        objectsFileStruct[isObject].quat_x = obLoc->quat_x[isObject]; //assign object quaternion x to struct
        objectsFileStruct[isObject].quat_y = obLoc->quat_y[isObject]; //assign object quaternion y to struct
        objectsFileStruct[isObject].quat_z = obLoc->quat_z[isObject]; //assign object quaternion z to struct
        objectsFileStruct[isObject].quat_w = obLoc->quat_w[isObject]; //assign object quaternion w to struct

        if (DEBUG_objectLocationsCallback) { //print off debug lines
            cout << "array element in id " << isObject << endl;
            cout << objectsFileStruct[isObject].id << "," <<
                    objectsFileStruct[isObject].object_name << ", " <<
                    objectsFileStruct[isObject].object_confidence << endl;
            cout << objectsFileStruct[isObject].point_x << ", " <<
                    objectsFileStruct[isObject].point_y << ", " <<
                    objectsFileStruct[isObject].point_z << endl;
            cout << objectsFileStruct[isObject].quat_x << ", " <<
                    objectsFileStruct[isObject].quat_y << ", " <<
                    objectsFileStruct[isObject].quat_z << ", " <<
                    objectsFileStruct[isObject].quat_w << endl;
            tofToolBox->printSeparator(0);
        }

        linkFileStruct[isObject].object_id = objectsFileStruct[isObject].id; //fill allocation spaces in numerical sequence - id
        linkFileStruct[isObject].object_name = objectsFileStruct[isObject].object_name; //fill allocation spaces in numerical sequence - name
    }
}

/**
 * Main callback function triggered by objects detected
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void detectedObjectsCallback(const wheelchair_msgs::objectLocations obLoc) {
    if (totalLinkFileStruct != 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); //wait 200 milliseconds in thread for all objects to update
        //run through entire list of objects
        //add object to struct
        //get ID of object and name
        //check to see if it already exists in this object struct
        //if it does update it
        //if it doesn't add it and assign it a room
        //eventually all objects should be present and have been assigned a room
        //publish the entire struct afterwards, to allow the navigation node to probe for object/room locations.
        int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg
        if (DEBUG_detectedObjectsCallback) {
            cout << "debug current room name is: " << currentRoomName << endl;
        }
        if (currentRoomName != "") {
            //room name detected
            //run through list of detected objects
            for (int isDetObject = 0; isDetObject < totalObjectsInMsg; isDetObject++) {
                if (DEBUG_detectedObjectsCallback) {
                    cout << "Current obj msg is " <<
                    obLoc.id[isDetObject] << ", " <<
                    obLoc.object_name[isDetObject] << endl;
                }
                int foundMatchingID = 0;
                int linkObjectID = 0;
                for (int isLinkObject = 0; isLinkObject < totalLinkFileStruct; isLinkObject++) {
                    if (linkFileStruct[isLinkObject].object_id == obLoc.id[isDetObject]) {
                        //found matchind IDs, allocate room id and name to correct memory space
                        if (DEBUG_detectedObjectsCallback) {
                            cout <<
                            "array space and object ID are: " <<
                            isLinkObject << ", " <<
                            obLoc.id[isDetObject] << endl;
                        }
                        foundMatchingID = 1;
                        linkObjectID = isLinkObject;
                    }
                    else {
                        //don't do anything - object ID not matched
                    }
                }
                if (foundMatchingID) {
                    if (DEBUG_foundMatchingID) {
                        cout <<
                        "found matching ID, allocating " <<
                        currentRoomName <<
                        " to " <<
                        linkFileStruct[linkObjectID].object_name <<
                        endl;
                    }
                    linkFileStruct[linkObjectID].room_id = currentRoomID;
                    linkFileStruct[linkObjectID].room_name = currentRoomName;
                }
                else {
                    if (DEBUG_foundMatchingID) {
                        cout <<
                        "something went wrong.... object not in full list\n" <<
                        "should have detected " <<
                        obLoc.id[isDetObject] <<
                        obLoc.object_name[isDetObject] << "\n" <<
                        "last item in full list was " <<
                        linkFileStruct[totalLinkFileStruct-1].object_id <<
                        linkFileStruct[totalLinkFileStruct-1].object_name <<
                        endl;
                    }
                }
            }
        }
    }
    else {
        //don't do anything yet, full list has not been provided
    }
}

/**
 * Main callback function triggered by objects detected
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void detectedObjectsCallback2(const wheelchair_msgs::objectLocations obLoc) {
    //add object to struct
    //get ID of object and name
    //check to see if it already exists in this object struct
    //if it does update it
    //if it doesn't add it and assign it a room
    //eventually all objects should be present and have been assigned a room
    //publish the entire struct afterwards, to allow the navigation node to probe for object/room locations.
    int totalObjectsInMsg = obLoc.totalObjects; //total detected objects in ROS msg
    int foundObjectMatch = 0;
    int matchPos = 0;
    if (DEBUG_detectedObjectsCallback) {
        cout << "debug current room name is: " << currentRoomName << endl;
    }
    if (currentRoomName != "") {
        for (int isObjectMsg = 0; isObjectMsg < totalObjectsInMsg; isObjectMsg++) { //run through ROS topic array
            if (DEBUG_detectedObjectsCallback) {
                cout << "Current obj msg is " << 
                obLoc.id[isObjectMsg] << ", " <<
                obLoc.object_name[isObjectMsg] << endl;
            }
            /*if (totalLinkFileStruct == 0) { //can't start for loop if struct is empty - so add some initial data
                linkFileStruct[totalRoomsFileStruct].object_id = obLoc.id[isObjectMsg]; //assign current msg object id to struct
                linkFileStruct[totalRoomsFileStruct].object_name = obLoc.object_name[isObjectMsg]; //assign current msg object name to struct

                linkFileStruct[totalRoomsFileStruct].room_id = currentRoomID; //add current room id to object struct
                linkFileStruct[totalRoomsFileStruct].room_name = currentRoomName; //add current room name to object struct
                totalLinkFileStruct++; //increase size of object struct array
            }*/
            for (int isObject = 0; isObject < totalLinkFileStruct; isObject++) {
                //run through struct for match
                if ((linkFileStruct[isObject].object_name == obLoc.object_name[isObjectMsg]) &&  //check to see if object name matches
                (linkFileStruct[isObject].object_id == obLoc.id[isObjectMsg])) { //check to see if object IDs match
                    if (DEBUG_detectedObjectsCallback) {
                        cout << "match found: " <<
                        linkFileStruct[isObject].object_name << ", " <<
                        obLoc.object_name[isObjectMsg] << endl;
                    }
                    foundObjectMatch = 1; //set found match var to true
                    matchPos = isObject; //save array pos of match to variable
                }
                else {
                    //don't do anything, if new obstacle is received, then foundObjectMatch remains 0
                }
            }
            if (foundObjectMatch) {
                //don't need to update object ID and name...
                //update the object to current room
                if (DEBUG_detectedObjectsCallback) {
                    cout << "updated existing object" << endl;
                }
                linkFileStruct[matchPos].room_id = currentRoomID;
                linkFileStruct[matchPos].room_name = currentRoomName;
            }
            else {
                //add object to struct and assign it a room
                if (DEBUG_detectedObjectsCallback) {
                    cout << "adding new object to struct" << endl;
                }
                linkFileStruct[totalLinkFileStruct].object_id = obLoc.id[isObjectMsg]; //assign current msg object id to struct
                linkFileStruct[totalLinkFileStruct].object_name = obLoc.object_name[isObjectMsg]; //assign current msg object name to struct
                linkFileStruct[totalLinkFileStruct].room_id = currentRoomID; //add current room id to object struct
                linkFileStruct[totalLinkFileStruct].room_name = currentRoomName; //add current room name to object struct
                totalLinkFileStruct++; //increase size of object struct array
            }
        }
    }
}

/**
 * Function gets current wheelchair pose and adds coordinates to struct
 *
 * @param parameter 'roomName_msg' is a string of the room name
 *        message belongs to std::string
 */
void addNewRoomToStruct(std::string roomName_msg) {
    roomsFileStruct[totalRoomsFileStruct].room_id = totalRoomsFileStruct; //add last room id to struct
    roomsFileStruct[totalRoomsFileStruct].room_name = roomName_msg; //add new room name to struct
    currentRoomID = roomsFileStruct[totalRoomsFileStruct].room_id; //set room id to current room
    currentRoomName = roomsFileStruct[totalRoomsFileStruct].room_name; //set as current room name

    //get the room location on the map
    int validTransform = 0;
    std::string map_frame = "/map";
    std::string robot_frame = "/base_link";
    tf::StampedTransform translation; //initiate translation for transform object
    while (validTransform == 0) { //keep looping if transform is not valid
        try {
            ptrListener->waitForTransform(map_frame, robot_frame, ros::Time(0), ros::Duration(3.0)); //wait a few seconds for ROS to respond
            ptrListener->lookupTransform(map_frame, robot_frame, ros::Time(), translation); //lookup translation of object from map frame

            //get global translation of object
            float translation_x = translation.getOrigin().x(); //set translation x to local variable
            float translation_y = translation.getOrigin().y(); //set translation y to local variable
            float translation_z = translation.getOrigin().z(); //set translation z to local variable
            float rotation_x = translation.getRotation().x(); //set rotation x to local variable
            float rotation_y = translation.getRotation().y(); //set rotation y to local variable
            float rotation_z = translation.getRotation().z(); //set rotation z to local variable
            float rotation_w = translation.getRotation().w(); //set rotation w to local variable

            if (DEBUG_roomNameCallback) {
                tofToolBox->printSeparator(0);
                //print out object name
                cout << roomsFileStruct[totalRoomsFileStruct].room_id << ", " <<
                        roomsFileStruct[totalRoomsFileStruct].room_name << endl;
                cout << translation_x << ", " <<
                        translation_y << ", " <<
                        translation_z << ", " <<
                        rotation_x << ", " <<
                        rotation_y << ", " <<
                        rotation_z << ", " <<
                        rotation_w << endl;
            }
            tf::Transform mapTransform;
            //create map transform from map to object frame
            mapTransform.setOrigin(
                    tf::Vector3(
                            translation_x,
                            translation_y,
                            translation_z) );
            tf::Quaternion mapQuaternion(
                    rotation_x,
                    rotation_y,
                    rotation_z,
                    rotation_w);
            mapTransform.setRotation(mapQuaternion);
            int foundNaN = tofToolBox->validateTransform(mapTransform);
            if (foundNaN) {
                //skip object
                cout << "NaN detected when transforming to map frame" << endl;
            }
            else {
                roomsFileStruct[totalRoomsFileStruct].point_x = translation_x;
                roomsFileStruct[totalRoomsFileStruct].point_y = translation_y;
                roomsFileStruct[totalRoomsFileStruct].point_z = translation_z;

                roomsFileStruct[totalRoomsFileStruct].quat_x = rotation_x;
                roomsFileStruct[totalRoomsFileStruct].quat_y = rotation_y;
                roomsFileStruct[totalRoomsFileStruct].quat_z = rotation_z;
                roomsFileStruct[totalRoomsFileStruct].quat_w = rotation_w;
                totalRoomsFileStruct++;
                validTransform = 1; //break out of while loop
            }
        }
        catch (tf::TransformException ex) {
            cout << "Couldn't get translation..." << endl; //catchment function if it can't get a translation from the map
            ROS_ERROR("%s",ex.what()); //print error
            ros::Duration(1.0).sleep();
        }
    }
}

/**
 * Main callback function triggered by received room name topic 
 *
 * @param parameter 'roomNameMsg' is a string of the room name from the wheelchair interface
 *        message belongs to std_msgs::String
 */
void roomNameCallback(const std_msgs::String roomNameMsg) {
    if (DEBUG_roomNameCallback) {
        tofToolBox->printSeparator(0);
        cout << "DEBUG_roomNameCallback" << endl;
    }
    std::string roomName_msg = roomNameMsg.data;
    int roomAlreadyExists = 0; //variable flag to see if room name is already present in struct

    int tempRoomID; //temporary room id in function scope
    std::string tempRoomName; //temporary room name in function scope

    if (DEBUG_roomNameCallback) {
        cout << "user input from topic is " << roomName_msg << endl; //this is working - something is going wrong further down in this function...
    }
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) { //run through entire rooms struct
        if (roomsFileStruct[isRoom].room_name == roomName_msg) { //if room name in struct is equal to room name from topic
            roomAlreadyExists = 1; //Room is already in rooms struct
            tempRoomID = roomsFileStruct[isRoom].room_id; //set struct room id to temp variable
            tempRoomName = roomsFileStruct[isRoom].room_name; //set struct room name to temp variable
        }
        else {
            //don't set anything, the room detected variable flag will remain at 0
        }
    }
    if (roomAlreadyExists) {
        currentRoomID = tempRoomID; //set temporary room id to current room id
        currentRoomName = tempRoomName; //set temporary room name to current room name
        //don't update room location struct - probably...
    }
    else {
        //add room not detected to struct
        addNewRoomToStruct(roomName_msg);
    }
    if (DEBUG_roomNameCallback) {
        cout << "current room id is " << currentRoomID << endl;
        cout << "current room name is " << currentRoomName << endl;
    }
}

/**
 * Function will broadcast room transforms 
 */
void broadcastRoomFrame() {
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) {
        //do stuff
        static tf::TransformBroadcaster br; //initialise broadcaster class
        //set frame name for room id and name
        std::string roomFrameName = std::to_string(roomsFileStruct[isRoom].room_id) + roomsFileStruct[isRoom].room_name;
        if (DEBUG_broadcastRoomFrame) {
            cout << "frame name is " << roomFrameName << endl;
        }

        tf::Transform mapTransform;
        mapTransform.setOrigin(
                tf::Vector3(
                        roomsFileStruct[isRoom].point_x,
                        roomsFileStruct[isRoom].point_y,
                        roomsFileStruct[isRoom].point_z) );

        tf::Quaternion mapQuaternion(
                roomsFileStruct[isRoom].quat_x,
                roomsFileStruct[isRoom].quat_y,
                roomsFileStruct[isRoom].quat_z,
                roomsFileStruct[isRoom].quat_w);
        mapTransform.setRotation(mapQuaternion);

        br.sendTransform(
                tf::StampedTransform(
                        mapTransform,
                        ros::Time::now(),
                        "map",
                        roomFrameName));

        //end the map frame object publishing
        if (DEBUG_broadcastRoomFrame) {
            cout << "publishing map frame" << endl;
        }
    }
}

/**
 * Function will publish all objects and associated rooms as a ROS msg array 
 */
void publishRoomsDacop() {
    wheelchair_msgs::roomToObjects room2Obj; //create ROS msg
    for (int isObject = 0; isObject < totalLinkFileStruct; isObject++) {
        room2Obj.object_id.push_back(linkFileStruct[isObject].object_id);
        room2Obj.object_name.push_back(linkFileStruct[isObject].object_name);
        room2Obj.room_id.push_back(linkFileStruct[isObject].room_id);
        room2Obj.room_name.push_back(linkFileStruct[isObject].room_name);
    }
    room2Obj.totalObjects = totalLinkFileStruct; //set total objects found in struct
    ptr_publish_roomsDacop->publish(room2Obj); //publish struct as ros msg array
}

/**
 * Function will publish all rooms stored in struct and publish as a ROS msg array 
 */
void publishRoomStruct() {
    wheelchair_msgs::roomLocations roomLoc; //create ROS msg
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) {
        roomLoc.id.push_back(roomsFileStruct[isRoom].room_id);
        roomLoc.room_name.push_back(roomsFileStruct[isRoom].room_name);

        roomLoc.point_x.push_back(roomsFileStruct[isRoom].point_x);
        roomLoc.point_y.push_back(roomsFileStruct[isRoom].point_y);
        roomLoc.point_z.push_back(roomsFileStruct[isRoom].point_z);

        roomLoc.quat_x.push_back(roomsFileStruct[isRoom].quat_x);
        roomLoc.quat_y.push_back(roomsFileStruct[isRoom].quat_y);
        roomLoc.quat_z.push_back(roomsFileStruct[isRoom].quat_z);
        roomLoc.quat_w.push_back(roomsFileStruct[isRoom].quat_w);
    }
    roomLoc.totalRooms = totalRoomsFileStruct; //set total rooms found in struct
    ptr_publish_rooms->publish(roomLoc); //publish struct as ros msg array
}

/**
 * Last function to save all rooms list struct data into files, ready for using on next startup 
 */
void saveRoomsList() {
    if (DEBUG_saveRoomsList) {
        cout << "saving all files" << endl;
    }
    //start with rooms.list
    ofstream FILE_WRITER;
	FILE_WRITER.open(rooms_list_loc);
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) {
        std::string roomListToSave = 
        std::to_string(roomsFileStruct[isRoom].room_id) + "," + 
        roomsFileStruct[isRoom].room_name + "," + 
        std::to_string(roomsFileStruct[isRoom].point_x) + "," + 
        std::to_string(roomsFileStruct[isRoom].point_y) + "," + 
        std::to_string(roomsFileStruct[isRoom].point_z) + "," + 
        std::to_string(roomsFileStruct[isRoom].quat_x) + "," + 
        std::to_string(roomsFileStruct[isRoom].quat_y) + "," + 
        std::to_string(roomsFileStruct[isRoom].quat_z) + "," + 
        std::to_string(roomsFileStruct[isRoom].quat_w) + "\n";

        FILE_WRITER << roomListToSave;
        if (DEBUG_saveRoomsList) {
            cout << roomListToSave;
        }
    }
    FILE_WRITER.close();
    if (DEBUG_saveRoomsList) {
        cout << "finished saving rooms.list" << endl;
    }
    //end with rooms.dacop
}

/**
 * Last function to save all object and room struct data into file rooms.dacop, ready for using on next startup 
 */
void saveRoomsDacop() {
    if (DEBUG_saveRoomsDacop) {
        cout << "saving rooms.dacop" << endl;
    }
    ofstream FILE_WRITER;
	FILE_WRITER.open(rooms_dacop_loc);
    for (int isObject = 0; isObject < totalLinkFileStruct; isObject++) {
        FILE_WRITER << 
        linkFileStruct[isObject].object_id << "," <<
        linkFileStruct[isObject].object_name << "," <<
        linkFileStruct[isObject].room_id << "," <<
        linkFileStruct[isObject].room_name << "\n";
        if (DEBUG_saveRoomsDacop) {
            cout << 
            linkFileStruct[isObject].object_id << "," <<
            linkFileStruct[isObject].object_name << "," <<
            linkFileStruct[isObject].room_id << "," <<
            linkFileStruct[isObject].room_name << "\n";
        }
    }
    FILE_WRITER.close();
    if (DEBUG_saveRoomsDacop) {
        cout << "finished saving rooms.dacop" << endl;
    }
}

/**
 * Main function that contains ROS info, subscriber callback trigger and while loop to get room name
 *
 * @param argc - number of arguments
 * @param argv - content of arguments
 * @return 0 - end of program
 */
int main (int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    //notes:
    //take UID from publish_objects_location and pass it through here
    //when msg comes through with UID of object - append a room name to the object
    wheelchair_dump_loc = tofToolBox->doesPkgExist("wheelchair_dump");//check to see if dump package exists

	rooms_list_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_list_name; //concatenate vars to create location of rooms list
    tofToolBox->createFile(rooms_list_loc); //check to see if file is present, if not create a new one

    rooms_dacop_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_dacop_name; //concatenate vars to create location of rooms dacop file
    tofToolBox->createFile(rooms_dacop_loc);

    roomListToStruct(rooms_list_loc);
    roomsDacopToStruct(rooms_dacop_loc);
    
    ros::init(argc, argv, "assign_room_to_object");
    ros::NodeHandle n;
    ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 1000, objectLocationsCallback); //full list of objects

    //delay object detected thread by 500 milliseconds, to allow the full objects list to be processed
    ros::NodeHandle n_delayThread;
    ros::CallbackQueue callback_queue_delayThread;
    n_delayThread.setCallbackQueue(&callback_queue_delayThread);
    ros::Subscriber detected_objects_sub = n_delayThread.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 1000, detectedObjectsCallback);
    std::thread spinner_thread_delay([&callback_queue_delayThread]() {
        ros::SingleThreadedSpinner spinner_delay;
        spinner_delay.spin(&callback_queue_delayThread);
    });

    //ros::Subscriber detected_objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 1000, detectedObjectsCallback);
    ros::Subscriber roomName_sub = n.subscribe("wheelchair_robot/user/room_name", 1000, roomNameCallback);
    //publish objects and associated rooms
    ros::Publisher local_publish_roomsDacop = n.advertise<wheelchair_msgs::roomToObjects>("wheelchair_robot/dacop/assign_room_to_object/objects", 1000);
    //publish rooms struct
    ros::Publisher local_publish_rooms = n.advertise<wheelchair_msgs::roomLocations>("wheelchair_robot/dacop/assign_room_to_object/rooms", 1000);
    ptr_publish_roomsDacop = &local_publish_roomsDacop; //point this local pub variable to global status, so the publish function can access it.
    ptr_publish_rooms = &local_publish_rooms; //point this local pub variable to global status
    tf::TransformListener listener; //listen to tf tree - to get translation of base_link against map
    ptrListener = &listener; //set to global pointer - to access from another function
    //publish associated object rooms from publish object locations
    //publish all of objects and rooms struct

    
    ros::Rate rate(10.0);
    while(ros::ok()) {
        //n.getParam("/wheelchair_robot/param/user/room_name", userRoomName);
        //cout << "room param is " << userRoomName << endl;
        broadcastRoomFrame();
        publishRoomsDacop();
        publishRoomStruct();
        
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
    saveRoomsList();
    saveRoomsDacop();
    return 0;
}