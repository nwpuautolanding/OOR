#include <iostream>
#include <cstdlib>

// ROS
#include <ros/ros.h>

// PCL
//~#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/conversions.h>
 #include <sensor_msgs/PointCloud2.h>
 #include <pcl_conversions/pcl_conversions.h>

 #include <iostream>
 #include <boost/thread/thread.hpp>
 #include <lobos_cloud_pubsub/cloudSubscriber.hpp>
#include <pcl/console/parse.h>
//variables for visualizarion custom
int r=0;
int g=0;
int b=255;
int psize=0.5;
typedef pcl::PointXYZRGBA PointT;

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

 

/** 
 */
int main(int argc, char **argv) {
  using namespace ros;
  ros::init(argc,argv,argv[1]);
  ros::NodeHandle nh;
  bool update = false;
  bool rgb=false;
  if (argc<3)
  {
	  printf("Incorrect number of argument ,usage: [std:: name of node to create][name of topic to subscribe]");
	  return 0;
  }
  
  
      
	  pcl::PointCloud<PointT>::Ptr ptCloud (new pcl::PointCloud<PointT>);
	  pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_p_cloud(ptCloud);
  
  	//~pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_p_cloud(ptCloud, r, g, b); 
	
  
       CloudSubscriber<PointT>  cs (nh, argv[2]);
   
 
   
   ros::AsyncSpinner spinner(1); // Use 1 threads
   
    
   spinner.start();
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
   viewer = createVis();
   while (!viewer->wasStopped ())
   {
            viewer->spinOnce (1);
            // Get lock on the boolean update and check if cloud was updated
            
            if(cs.getIsThereNewData())
            {
				*ptCloud = cs.getCurrentPointclout();
				
                if(!viewer->updatePointCloud(ptCloud,color_p_cloud, "sample cloud"))
                  viewer->addPointCloud(ptCloud, color_p_cloud, "sample cloud");
                
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "sample cloud");
                
            }
            

     } 
  
  
  
    printf("despues de ros spin");	
    spinner.stop();
   //~ros::waitForShutdown();
  
   
	return 0;
}
