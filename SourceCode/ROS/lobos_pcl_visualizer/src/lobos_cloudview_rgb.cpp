#include <iostream>
#include <cstdlib>

// ROS
#include <ros/ros.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <lobos_cloud_pubsub/CloudSubscriber.hpp>
#include <pcl/console/parse.h>
//defintion of the type fo point cloud to be used in the visualizer
typedef pcl::PointXYZRGBA PointT;
using namespace std;
int psize = 0.5;
boost::shared_ptr<pcl::visualization::PCLVisualizer> createVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer and add properties-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Lobos' Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters ();
  return (viewer);
  
}

int main(int argc, char **argv) {

  if (argc<1)
  {
	  printf("Incorrect number of arguments ,usage: [optional std::string name of node to create][input:= 'name of topic of the XYZRGBA PC to subscribe']");
	  return 0;
  }
  using namespace ros;
  string node_name="RGBvisualizer";
  if(argc<2)
  {
	  node_name = argv[1];
  }
  ros::init(argc,argv,node_name);
  ros::NodeHandle nh;
  bool update = false;
  bool rgb=false;
  
  
  pcl::PointCloud<PointT>::Ptr ptCloud (new pcl::PointCloud<PointT>);
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_p_cloud(ptCloud);
  CloudSubscriber<PointT>  cs (nh, "input");
  ros::AsyncSpinner spinner(1); // Use 1 threads
  spinner.start();
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = createVis();
  while (!viewer->wasStopped ())
   {
            viewer->spinOnce (1000);
            // Get lock on the boolean update and check if cloud was updated
            
            if(cs.getIsThereNewData())
            {
				*ptCloud = cs.getCurrentPointclout();
				
                if(!viewer->updatePointCloud(ptCloud,color_p_cloud, "sample cloud"))
                  viewer->addPointCloud(ptCloud, color_p_cloud, "sample cloud");
                
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "sample cloud");
            }
        } 
    spinner.stop();
   	return 0;
}
