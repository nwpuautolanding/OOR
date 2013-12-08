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

// Default variables for visualizarion custom
int r=0;
int g=0;
int b=255;
int psize=0.1;
//defintion of the type fo point cloud to be used in the visualizer
typedef pcl::PointXYZ PointT;

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
  using namespace ros;
  //Ros intializaton, arg[1] is the name that will be givne to the node, this is useful when running several instances of the node, remember that twio nodes with the same name can't run
  
  ros::init(argc,argv,argv[1]);
  ros::NodeHandle nh;
  
  if (argc<1)
  {
	  printf("Incorrect number of arguments ,usage: [std::string name of node to create][optional  r g b values for visualization of the point cloud][input = std::string::name of topic of the XYZ PC  to subscribe]");
	  return 0;
  }
  if (argc>3)
  {
	  r= atoi(argv[3]);
	  g= atoi(argv[4]);
	  b= atoi(argv[5]);
  }
  pcl::PointCloud<PointT>::Ptr ptCloud (new pcl::PointCloud<PointT>);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_p_cloud(ptCloud, r, g, b); 
  //~CloudSubscriber<PointT>  cs (nh, argv[2]);
  CloudSubscriber<PointT>  cs (nh, "input");
  ros::AsyncSpinner spinner(1); // Use 1 threads
  spinner.start();
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = createVis();
   while (!viewer->wasStopped ())
   {
            viewer->spinOnce (1);
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
