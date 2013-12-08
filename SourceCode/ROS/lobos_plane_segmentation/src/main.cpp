#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <lobos_cloud_pubsub/cloudSubscriber.hpp>
#include <string>

int main (int argc, char **argv) {
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    CloudSubscriber<pcl::PointXYZ>  cs (nh, std::string("hola"));

    ros::spin();
    return 0;
}
