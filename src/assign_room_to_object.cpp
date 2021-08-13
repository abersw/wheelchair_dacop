/*
 * assign_room_to_object.cpp
 * wheelchair_dacop
 * version: 0.1.0 Majestic Maidenhair
 * Status: Alpha
*/

#include "tof_tool/tof_tool_box.h"

using namespace std;

const int DEBUG_createFile = 0;
const int DEBUG_roomListToStruct = 0;
const int DEBUG_roomsDacopToStruct = 0;
const int DEBUG_objectLocationsCallback = 0;
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
                    objectsFileStruct[objectNumber].object_id = std::stoi(token); //convert object id string to int
                }
                else if (lineSection == 1) { //if second delimiter
                    objectsFileStruct[objectNumber].object_name = token; //get object name
                }
                else if (lineSection == 2) { //if third delimiter
                    objectsFileStruct[objectNumber].room_id = std::stoi(token); //convert room id string to int
                }
                lineSection++; //go to next delimiter
            }
            objectsFileStruct[objectNumber].room_name = line; //get end of line - room name
            if (DEBUG_roomsDacopToStruct) {
                cout << 
                objectsFileStruct[objectNumber].object_id << "," <<
                objectsFileStruct[objectNumber].object_name << "," <<
                objectsFileStruct[objectNumber].room_id << "," <<
                objectsFileStruct[objectNumber].room_name << endl;
            }
            objectNumber++; //go to next object line
        }
    }
    totalObjectsFileStruct = objectNumber; //set total number of objects in struct to the lines(obstacles) counted
}

/**
 * Main callback function triggered by received ROS topic 
 *
 * @param parameter 'obLoc' is the array of messages from the publish_object_locations node
 *        message belongs to wheelchair_msgs objectLocations.msg
 */
void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
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
    if (DEBUG_objectLocationsCallback) {
        cout << "debug current room name is: " << currentRoomName << endl;
    }
    if (currentRoomName != "") {
        for (int isObjectMsg = 0; isObjectMsg < totalObjectsInMsg; isObjectMsg++) { //run through ROS topic array
            if (DEBUG_objectLocationsCallback) {
                cout << "Current obj msg is " << 
                obLoc.id[isObjectMsg] << ", " <<
                obLoc.object_name[isObjectMsg] << endl;
            }
            /*if (totalObjectsFileStruct == 0) { //can't start for loop if struct is empty - so add some initial data
                objectsFileStruct[totalRoomsFileStruct].object_id = obLoc.id[isObjectMsg]; //assign current msg object id to struct
                objectsFileStruct[totalRoomsFileStruct].object_name = obLoc.object_name[isObjectMsg]; //assign current msg object name to struct

                objectsFileStruct[totalRoomsFileStruct].room_id = currentRoomID; //add current room id to object struct
                objectsFileStruct[totalRoomsFileStruct].room_name = currentRoomName; //add current room name to object struct
                totalObjectsFileStruct++; //increase size of object struct array
            }*/
            for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
                //run through struct for match
                if ((objectsFileStruct[isObject].object_name == obLoc.object_name[isObjectMsg]) &&  //check to see if object name matches
                (objectsFileStruct[isObject].object_id == obLoc.id[isObjectMsg])) { //check to see if object IDs match
                    if (DEBUG_objectLocationsCallback) {
                        cout << "match found: " <<
                        objectsFileStruct[isObject].object_name << ", " <<
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
                if (DEBUG_objectLocationsCallback) {
                    cout << "updated existing object" << endl;
                }
                objectsFileStruct[matchPos].room_id = currentRoomID;
                objectsFileStruct[matchPos].room_name = currentRoomName;
            }
            else {
                //add object to struct and assign it a room
                if (DEBUG_objectLocationsCallback) {
                    cout << "adding new object to struct" << endl;
                }
                objectsFileStruct[totalObjectsFileStruct].object_id = obLoc.id[isObjectMsg]; //assign current msg object id to struct
                objectsFileStruct[totalObjectsFileStruct].object_name = obLoc.object_name[isObjectMsg]; //assign current msg object name to struct
                objectsFileStruct[totalObjectsFileStruct].room_id = currentRoomID; //add current room id to object struct
                objectsFileStruct[totalObjectsFileStruct].room_name = currentRoomName; //add current room name to object struct
                totalObjectsFileStruct++; //increase size of object struct array
            }
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
    int roomDetected = 0; //variable flag to see if room name is already present in struct

    int tempRoomID; //temporary room id in function scope
    std::string tempRoomName; //temporary room name in function scope

    if (DEBUG_roomNameCallback) {
        cout << "user input from topic is " << roomName_msg << endl; //this is working - something is going wrong further down in this function...
    }
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) { //run through entire rooms struct
        if (roomsFileStruct[isRoom].room_name == roomName_msg) { //if room name in struct is equal to room name from topic
            roomDetected = 1; //Room is already in rooms struct
            tempRoomID = roomsFileStruct[isRoom].room_id; //set struct room id to temp variable
            tempRoomName = roomsFileStruct[isRoom].room_name; //set struct room name to temp variable
        }
        else {
            //don't set anything, the room detected variable flag will remain at 0
        }
    }
    if (roomDetected) {
        currentRoomID = tempRoomID; //set temporary room id to current room id
        currentRoomName = tempRoomName; //set temporary room name to current room name
        //don't update room location struct - probably...
    }
    else {
        //add room not detected to struct
        roomsFileStruct[totalRoomsFileStruct].room_id = totalRoomsFileStruct; //add last room id to struct
        roomsFileStruct[totalRoomsFileStruct].room_name = roomName_msg; //add new room name to struct
        currentRoomID = roomsFileStruct[totalRoomsFileStruct].room_id; //set room id to current room
        currentRoomName = roomsFileStruct[totalRoomsFileStruct].room_name; //set as current room name

        //get the room location on the map
        std::string robot_frame = "/base_link";
        tf::StampedTransform translation; //initiate translation for transform object
        try {
            ptrListener->waitForTransform("/map", robot_frame, ros::Time(0), ros::Duration(3.0)); //wait a few seconds for ROS to respond
            ptrListener->lookupTransform("/map", robot_frame, ros::Time(), translation); //lookup translation of object from map frame

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
                cout << roomsFileStruct[totalRoomsFileStruct].room_id << ", " << roomsFileStruct[totalRoomsFileStruct].room_name << endl; //print out object name
                cout << translation_x << ", " << translation_y << ", " << translation_z << ", " << rotation_x << ", " << rotation_y << ", " << rotation_z << ", " << rotation_w << endl;
            }

            roomsFileStruct[totalRoomsFileStruct].point_x = translation_x;
            roomsFileStruct[totalRoomsFileStruct].point_y = translation_y;
            roomsFileStruct[totalRoomsFileStruct].point_z = translation_z;

            roomsFileStruct[totalRoomsFileStruct].quat_x = rotation_x;
            roomsFileStruct[totalRoomsFileStruct].quat_y = rotation_y;
            roomsFileStruct[totalRoomsFileStruct].quat_z = rotation_z;
            roomsFileStruct[totalRoomsFileStruct].quat_w = rotation_w;
        }
        catch (tf::TransformException ex) {
            cout << "Couldn't get translation..." << endl; //catchment function if it can't get a translation from the map
            ROS_ERROR("%s",ex.what()); //print error
            ros::Duration(1.0).sleep();
        }

        totalRoomsFileStruct++;
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
        std::string roomFrameName = std::to_string(roomsFileStruct[isRoom].room_id) + roomsFileStruct[isRoom].room_name; //set frame name for room id and name
        if (DEBUG_broadcastRoomFrame) {
            cout << "frame name is " << roomFrameName << endl;
        }

        tf::Transform mapTransform;
        mapTransform.setOrigin( tf::Vector3(roomsFileStruct[isRoom].point_x, roomsFileStruct[isRoom].point_y, roomsFileStruct[isRoom].point_z) );

        tf::Quaternion mapQuaternion(roomsFileStruct[isRoom].quat_x, roomsFileStruct[isRoom].quat_y, roomsFileStruct[isRoom].quat_z, roomsFileStruct[isRoom].quat_w);
        mapTransform.setRotation(mapQuaternion);
        br.sendTransform(tf::StampedTransform(mapTransform, ros::Time::now(), "map", roomFrameName));

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
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
        room2Obj.object_id.push_back(objectsFileStruct[isObject].object_id);
        room2Obj.object_name.push_back(objectsFileStruct[isObject].object_name);
        room2Obj.room_id.push_back(objectsFileStruct[isObject].room_id);
        room2Obj.room_name.push_back(objectsFileStruct[isObject].room_name);
    }
    room2Obj.totalObjects = totalObjectsFileStruct; //set total objects found in struct
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
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
        FILE_WRITER << 
        objectsFileStruct[isObject].object_id << "," <<
        objectsFileStruct[isObject].object_name << "," <<
        objectsFileStruct[isObject].room_id << "," <<
        objectsFileStruct[isObject].room_name << "\n";
        if (DEBUG_saveRoomsDacop) {
            cout << 
            objectsFileStruct[isObject].object_id << "," <<
            objectsFileStruct[isObject].object_name << "," <<
            objectsFileStruct[isObject].room_id << "," <<
            objectsFileStruct[isObject].room_name << "\n";
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
    createFile(rooms_list_loc); //check to see if file is present, if not create a new one

    rooms_dacop_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_dacop_name; //concatenate vars to create location of rooms dacop file
    createFile(rooms_dacop_loc);

    roomListToStruct(rooms_list_loc);
    roomsDacopToStruct(rooms_dacop_loc);
    ros::init(argc, argv, "assign_room_to_object");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/detected_objects", 10, objectLocationsCallback);
    ros::Subscriber roomName_sub = n.subscribe("wheelchair_robot/user/room_name", 10, roomNameCallback);
    ros::Publisher local_publish_roomsDacop = n.advertise<wheelchair_msgs::roomToObjects>("wheelchair_robot/dacop/assign_room_to_object/objects", 1000); //publish objects and associated rooms
    ros::Publisher local_publish_rooms = n.advertise<wheelchair_msgs::roomLocations>("wheelchair_robot/dacop/assign_room_to_object/rooms", 1000);//publish rooms struct
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