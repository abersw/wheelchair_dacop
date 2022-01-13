/*
 * missing_objects.cpp
 * wheelchair_context
 * version: 0.0.1 Majestic Maidenhair
 * Status: Alpha
 *
*/

#include "tof_tool/tof_tool_box.h"

#include "wheelchair_msgs/objectLocations.h"

using namespace std;

static const int DEBUG_main = 1;

TofToolBox *tofToolBox;

void objectLocationsCallback(const wheelchair_msgs::objectLocations obLoc) {
    //add code
}

int main (int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    //take UID from publish_objects_location and pass it through here
    //when msg comes through with ID of object - append a room name to the object
    ros::init(argc, argv, "missing_objects");
    ros::NodeHandle n;

    //subscribe to full list of objects from central public_object_locations node
    ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 1000, objectLocationsCallback);

    ros::Rate rate(10.0);


    while(ros::ok()) {
        //tofToolBox->sayHello(); //test function for tof toolbox
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
}
