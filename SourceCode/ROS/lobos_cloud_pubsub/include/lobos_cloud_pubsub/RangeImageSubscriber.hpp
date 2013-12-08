
#ifndef RANGE_IMAGE_SUBSCRIBER
#define RANGE_IMAGE_SUBSCRIBER

#include <string>
#include <ros/ros.h>
#include <pcl/range_image/range_image_planar.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

class RangeImageSubscriber {
        
private:
    /**
    * Attributes
    */
    // Attribute where the point cloud is going to be saved
    pcl::RangeImagePlanar localRangeImage;
    sensor_msgs::CameraInfoConstPtr localImageInfo;
    sensor_msgs::ImageConstPtr localDepthImage;
    // Ros subscriber
    ros::Subscriber rangeImageSub;
    ros::Subscriber cameraInfoSub;
    bool isThereNewRangeImage;
    bool isThereNewCameraInfo;
    bool isThereNewDepthImage;
    float angularResolution;

    //Private methods
    void depthImageCallback (const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback (const sensor_msgs::CameraInfoConstPtr& msg);
    void computeRangeImage ();
public:
    RangeImageSubscriber (ros::NodeHandle nh, std::string imageRangeTopicName, std::string cameraInfoTopicName);
    ~RangeImageSubscriber ();
    
    /**
     * Getters
     */
    pcl::RangeImagePlanar getCurrentRangeImage ();
    bool getIsThereNewData ();

};


#endif
