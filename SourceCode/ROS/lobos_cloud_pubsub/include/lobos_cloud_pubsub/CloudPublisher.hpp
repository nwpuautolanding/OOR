
#ifndef CLOUD_PUBLISHER
#define CLOUD_PUBLISHER

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

template <typename T>
class CloudPublisher {
        
private:
    /**
    * Attributes
    */
    // Ros publisher
    ros::Publisher pub;

public:
    CloudPublisher (ros::NodeHandle nh,std::string topicName);
    ~CloudPublisher();
    
    /**
     * Actions
     */
    void publishPointcloud (pcl::PointCloud<T> pc);

};

template <typename T>
CloudPublisher<T>::CloudPublisher (ros::NodeHandle nh, std::string topicName) {
    
    pub = nh.advertise<sensor_msgs::PointCloud2> (topicName, 5);
    
}

template <typename T>
CloudPublisher<T>::~CloudPublisher() {

}


/**
* Actions
*/

template <typename T>
void CloudPublisher<T>::publishPointcloud(pcl::PointCloud<T> pc) {

    sensor_msgs::PointCloud2 rosCloud;
    pcl::toROSMsg(pc, rosCloud);
    pub.publish (rosCloud);

}

#endif
