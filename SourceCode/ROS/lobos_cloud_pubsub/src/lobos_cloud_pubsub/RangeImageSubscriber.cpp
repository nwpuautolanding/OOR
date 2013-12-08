

#include <iostream>
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

    boost::lock_guard<boost::mutex> lock(my_mutex);
    localDepthImage = msg;
    isThereNewDepthImage = true;
    
    computeRangeImage ();

}

void RangeImageSubscriber::cameraInfoCallback (const sensor_msgs::CameraInfoConstPtr& msg) {

    boost::lock_guard<boost::mutex> lock(my_mutex);
    localImageInfo = msg;
    isThereNewCameraInfo = true;
    
    computeRangeImage ();

}

void RangeImageSubscriber::computeRangeImage() {

    
    if (isThereNewCameraInfo && isThereNewDepthImage) {
        isThereNewRangeImage = false;
        isThereNewCameraInfo = false;
        isThereNewRangeImage = true;

        //std::cout << "localDepthimagesize " << localDepthImage->height << " " << localDepthImage->width << std::endl;
	

        localRangeImage.setDepthImage(reinterpret_cast<const float*> (&localDepthImage->data[0]),
	                                  (int)localDepthImage->width, (int)localDepthImage->height,
                                      //3.3930780975300314e+02, 2.4273913761751615e+02,
                                      //5.9421434211923247e+02, 5.9104053696870778e+02, angularResolution);

	                                  (float)localImageInfo->P[2],  (float)localImageInfo->P[6],
	                                  (float)localImageInfo->P[0],  (float)localImageInfo->P[5]);

        //std::cout << "Image infog: ";
        //for (int i = 0; i < 12; i++) {
      //    std::cout << localImageInfo->P[i] << " ";
      //}
      //std::cout << std::endl;
      //std::cout << "localRangeImagesize: " << localRangeImage.height << " " << localRangeImage.width << std::endl;
    }

}

/**
* Getters
*/

pcl::RangeImagePlanar RangeImageSubscriber::getCurrentRangeImage() {

    boost::lock_guard<boost::mutex> lock(my_mutex);
    isThereNewRangeImage = false;
    pcl::RangeImagePlanar tmp;
    localRangeImage.copyTo(tmp);
    return tmp;

}

bool RangeImageSubscriber::getIsThereNewData () {

    boost::lock_guard<boost::mutex> lock(my_mutex);
    return isThereNewRangeImage;

}
