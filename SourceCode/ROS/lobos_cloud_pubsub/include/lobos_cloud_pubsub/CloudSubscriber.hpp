
#ifndef CLOUD_SUBSCRIBER
#define CLOUD_SUBSCRIBER

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <boost/thread.hpp>

template <typename T>
class CloudSubscriber {
        
private:
    /**
    * Attributes
    */
    // Attribute where the point cloud is going to be saved
    pcl::PointCloud<T> localPointCloud;
    // Ros subscriber
    ros::Subscriber sub;
    bool isThereNewData;
    boost::mutex my_mutex;

    //Private methods
    void callback (const sensor_msgs::PointCloud2ConstPtr& input);

public:
    CloudSubscriber (ros::NodeHandle nh,std::string topicName);
    ~CloudSubscriber();
    
    /**
     * Getters
     */
    pcl::PointCloud<T> getCurrentPointclout ();
    bool getIsThereNewData();

};

template <typename T>
CloudSubscriber<T>::CloudSubscriber (ros::NodeHandle nh, std::string topicName) {
    
    isThereNewData = false;
    sub = nh.subscribe (topicName, 1, &CloudSubscriber<T>::callback, this);
    
}

template <typename T>
CloudSubscriber<T>::~CloudSubscriber() {

}

template <typename T>
void CloudSubscriber<T>::callback (const sensor_msgs::PointCloud2ConstPtr& input) {

    boost::lock_guard<boost::mutex> lock(my_mutex);
   //localPointCloud = pcl::fromROSMsg (input);
   pcl::fromROSMsg<T>(*input, localPointCloud);
   isThereNewData = true;

}

/**
* Getters
*/

template <typename T>
pcl::PointCloud<T> CloudSubscriber<T>::getCurrentPointclout() {

    boost::lock_guard<boost::mutex> lock(my_mutex);
    isThereNewData = false;
    return localPointCloud;

}

template <typename T>
bool CloudSubscriber<T>::getIsThereNewData () {

    boost::lock_guard<boost::mutex> lock(my_mutex);
    return isThereNewData;

}

#endif
