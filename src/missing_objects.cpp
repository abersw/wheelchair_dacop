/*
 * missing_objects.cpp
 * wheelchair_context
 * version: 0.0.1 Majestic Maidenhair
 * Status: Alpha
 *
*/

#include "tof_tool/tof_tool_box.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "wheelchair_msgs/objectLocations.h"

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

using namespace std;

static const int DEBUG_main = 1;

TofToolBox *tofToolBox;

ros::Publisher pcl_pub;

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void objectLocationsCallback(const sensor_msgs::PointCloud2::ConstPtr& dpth, const wheelchair_msgs::objectLocations::ConstPtr& obLoc) {
    //convert ros pointcloud2 to pcl
    //https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/
    //create convex hull inside pcl
    //https://pcl.readthedocs.io/projects/tutorials/en/latest/hull_2d.html

    /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZRGB> cloud;
    //STEP 0 Convert sensor_msgs to pcl
    pcl::fromROSMsg(*dpth, cloud);

    pcl::ConvexHull<pcl::PointXYZ> cHull;
    pcl::PointCloud<pcl::PointXYZ> cHull_points;
    cHull.setInputCloud(cloud);
    cHull.reconstruct (cHull_points);*/

/*
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*dpth, pcl_pc2);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                    cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                    cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    // proj.setIndices (inliers);
    proj.setInputCloud (cloud_filtered);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    std::cerr << "PointCloud after projection has: "
            << cloud_projected->size () << " data points." << std::endl;

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    //chull.setAlpha (0.1);
    chull.reconstruct (*cloud_hull);
    std::cerr << "Concave hull has: " << cloud_hull->size ()
            << " data points." << std::endl;

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    //pcl::toROSMsg(cloud_filtered, output);

    // Publish the data
    //pcl_pub.publish (output);
    */

   //calculate field of view
   //iterate through pointcloud and find nearest point to transform, probably in xy coordinate
   //see if transform exists within a range
}

int main (int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    //take UID from publish_objects_location and pass it through here
    //when msg comes through with ID of object - append a room name to the object
    ros::init(argc, argv, "missing_objects");
    ros::NodeHandle n;

    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "/zed/zed_node/point_cloud/cloud_registered", 100); //get transformed pointcloud
    message_filters::Subscriber<wheelchair_msgs::objectLocations> objects_sub(n, "wheelchair_robot/dacop/publish_object_locations/objects", 100); //get mobilenet objects detected
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, wheelchair_msgs::objectLocations> MySyncPolicy; //approximately sync the topic rate
    message_filters::Synchronizer<MySyncPolicy> depth_sync(MySyncPolicy(20), depth_sub, objects_sub); //set sync policy
    depth_sync.registerCallback(boost::bind(&objectLocationsCallback, _1, _2)); //set callback for synced topics

    //subscribe to full list of objects from central public_object_locations node
    //ros::Subscriber objects_sub = n.subscribe("wheelchair_robot/dacop/publish_object_locations/objects", 1000, objectLocationsCallback);

    // Create a ROS publisher for the output point cloud
    pcl_pub = n.advertise<sensor_msgs::PointCloud2> ("/pcl/convexhull", 1000);

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
