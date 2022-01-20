/*
 * missing_objects.cpp
 * wheelchair_context
 * version: 0.0.1 Majestic Maidenhair
 * Status: Alpha
 *
*/

#include "tof_tool/tof_tool_box.h"


#include "wheelchair_msgs/objectLocations.h"
#include <sensor_msgs/PointCloud2.h>

using namespace std;

static const int DEBUG_main = 1;

TofToolBox *tofToolBox;

ros::Publisher pcl_pub;

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void objectLocationsCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::objectLocations::ConstPtr& obLoc) {
   //calculate field of view
   //calcualte the orientation of the robot, filter transforms within field of view of the robot
   //iterate through pointcloud and find nearest point to transform, probably in xy coordinate
   //see if transform exists within a range
}

int main (int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    ros::init(argc, argv, "missing_objects");
    ros::NodeHandle n;

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
