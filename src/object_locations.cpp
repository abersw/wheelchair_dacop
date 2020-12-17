#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include <ros/package.h> //find ROS packages, needs roslib dependency
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
#include <sqlite3.h>



#include <sstream>
using namespace std;
//using namespace message_filters;
//using namespace sensor_msgs;

sqlite3* DB;
std::string wheelchair_dump_loc;

struct Objects { //struct for publishing topic
    int id;
    string object_name;
    float point_x;
    float point_y;
    float point_z;

    float rotation_r;
    float rotation_p;
    float rotation_y;
};
int totalObjects;

void createAndBuildDatabase() {

    int DBerror = 0;
    std::string DBfileNameTmp = wheelchair_dump_loc + "/dump/dacop/objects.db";
    const char * DBfileName = DBfileNameTmp.c_str();
    cout << DBfileName << endl;
    DBerror = sqlite3_open(DBfileName, &DB);
    if (DBerror) {
        cout << "Could not find db\n";
        cerr << "Error open DB " << sqlite3_errmsg(DB) << std::endl;
        ofstream MyFile(DBfileName);
        MyFile.close();
        cout << "Created new DB file\n";
    }
    else {
        cout << "DB ok" << endl;
    }

    DBerror = sqlite3_open(DBfileName, &DB);
    //cout << "reached db open" << endl;
    //const char *mySqlTable;
    std::string mySqlTable;
    //create table inside database
    mySqlTable = "CREATE TABLE IF NOT EXISTS OBJECTS("  \
    "ID INT PRIMARY KEY  NOT NULL," \
    "NAME  VARCHAR(500)  NOT NULL," \
    "POINTX  DOUBLE  NOT NULL," \
    "POINTY  DOUBLE  NOT NULL," \
    "POINTZ  DOUBLE  NOT NULL," \
    "QUATX DOUBLE NOT NULL," \
    "QUATY DOUBLE NOT NULL," \
    "QUATZ DOUBLE NOT NULL," \
    "QUATW DOUBLE NOT NULL);";

    char* SQLerror;
    DBerror = sqlite3_exec(DB, mySqlTable.c_str(), NULL, 0, &SQLerror);

    if (DBerror != SQLITE_OK) {
        cerr << "Error Create Table" << std::endl;
        sqlite3_free(SQLerror);
    }
    else {
        cout << "Table created Successfully" << std::endl;
    }
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "object_depth");
    ros::NodeHandle n;

    ros::Rate rate(10.0);
    //ros::Subscriber sub = nh.subscribe("wheelchair_robot/something", 1, myCallback);
    wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");

    createAndBuildDatabase();


    //set global variable for file/database
    //if does not exist - create one
    //if using a database and table does not exist - create one
    
    if (ros::isShuttingDown()) {
        //close things safely
    }
    cout << "spin \n";
    ros::spin();
    rate.sleep();

    return 0;
}