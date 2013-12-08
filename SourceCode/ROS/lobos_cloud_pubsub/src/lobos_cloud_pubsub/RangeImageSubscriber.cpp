#include <lobos_cloud_pubsub/RangeImageSubscriber.hpp>

RangeImageSubscriber::RangeImageSubscriber (ros::NodeHandle nh, std::string imageRangeTopicName, std::string cameraInfoTopicName) {
    
    isThereNewDepthImage = false;
    isThereNewRangeImage = false;
    isThereNewCameraInfo = false;
    angularResolution = 0.5f;
    rangeImageSub = nh.subscribe (imageRangeTopicName, 1, &RangeImageSubscriber::depthImageCallback, this);
    cameraInfoSub= nh.subscribe (cameraInfoTopicName, 1, &RangeImageSubscriber::cameraInfoCallback, this);
    
}

RangeImageSubscriber::~RangeImageSubscriber() {

}

void RangeImageSubscriber::depthImageCallback (const sensor_msgs::ImageConstPtr& msg) {

    localDepthImage = msg;
    isThereNewDepthImage = true;
    
    computeRangeImage ();

}

void RangeImageSubscriber::cameraInfoCallback (const sensor_msgs::CameraInfoConstPtr& msg) {

    localImageInfo = msg;
    isThereNewCameraInfo = true;
    
    computeRangeImage ();

}

void RangeImageSubscriber::computeRangeImage() {

    if (isThereNewCameraInfo && isThereNewDepthImage) {
	isThereNewRangeImage = false;
        isThereNewCameraInfo = false;
	isThereNewRangeImage = true;
	
        localRangeImage.setDepthImage(reinterpret_cast<const float*> (&localDepthImage->data[0]),
	                                  localDepthImage->width, localDepthImage->height,
	                                  localImageInfo->P[2],  localImageInfo->P[6],
	                                  localImageInfo->P[0],  localImageInfo->P[5], angularResolution);
    }

}

/**
* Getters
*/

pcl::RangeImagePlanar RangeImageSubscriber::getCurrentRangeImage() {

    isThereNewRangeImage = false;
    return localRangeImage;

}

bool RangeImageSubscriber::getIsThereNewData () {

   return isThereNewRangeImage;

}
