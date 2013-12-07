
#ifndef CLOUD_SUBSCRIBER
#define CLOUD_SUBSCRIBER

#include <string>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>

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

    //Private methods
    void callback (const sensor_msgs::PointCloud2ConstPtr& input);

public:
    CloudSubscriber (ros::NodeHandle nh);
    ~CloudSubscriber();
    
    /**
     * Getters
     */
    pcl::PointCloud<T> getCurrentPointclout ();

}

template <typename T>
CloudSubscriber<T>::CloudSubscriber (ros::NodeHandle nh, std::string topicName) {
    
    sub = nh.subscribe (topicName, 1, &CloudSubscriber::callback, this);
    
}

template <typename T>
CloudSubscriber<t>::~CloudSubscriber() {

}

template <typename T>
void CloudSubscriber::callback (const sensor_msgs::PointCloud2ConstPtr& input) {

   localPointCloud = pcl::fromROSMsg (input);

}

template <typename T>
pcl::PointCloud<T> CloudSubscriber<T>::getCurrentPointcloud() {

    retunr localPointCloud;
}


#endif
