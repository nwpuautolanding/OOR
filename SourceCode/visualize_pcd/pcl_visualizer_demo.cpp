/* \author Geoffrey Biggs */

#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <stdlib.h>
using namespace std;
using namespace pcl;

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] [filename.pcd ][if -c add r g b values for custom diplay][pointsize (only -r and  -c options)]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation\n"
            << "-r           RGB colour visualisation\n"
            << "-c           Custom colour visualisation\n"
            << "-n           Normals visualisation\n"
            << "-a           Shapes visualisation\n"
             << "\n\n";
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Lobos Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, int psize)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Lobos Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "sample cloud");
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int r, int g, int b, int psize)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Lobos Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, r, g, b);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "sample cloud");
  viewer->initCameraParameters ();
  return (viewer);
}




// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
   if (argc < 3)
        {
            printUsage(argv[0]);
            return 0;
        }
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool simple(false), rgb(false), custom_c(false), normals(false),
    shapes(false), viewports(false), interaction_customization(false);
  int rr,gg,bb, point_size;
  if (pcl::console::find_argument (argc, argv, "-s") >= 0)
  {
    simple = true;
    std::cout << "Simple visualisation \n";
  }
  else if (pcl::console::find_argument (argc, argv, "-c") >= 0)
  {
    custom_c = true;
    std::cout << "Custom colour visualisation \n";
    rr= atoi(argv[3]);
    gg= atoi(argv[4]);
    bb=atoi(argv[5]);
    point_size=atoi(argv[6]);
    
  }
  else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
  {
	point_size= atoi(argv[3]);
    rgb = true;
    std::cout << "RGB colour visualisation e\n";
  }
   
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  // ------------------------------------
  // -----Read pcd File----
  // ------------------------------------
  PointCloud<PointXYZRGBA>::Ptr color_ptr(new PointCloud<PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_color_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<PointXYZRGBA> colorcloud;
  string filename=argv[2];
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (simple)
  {
	  
	io::loadPCDFile<PointXYZ>(filename.c_str(), *basic_cloud_ptr);
    viewer = simpleVis(basic_cloud_ptr);
    
  }
  else if (rgb)
  {
	 
	   try
        {
			
            pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename.c_str(), *pcl_color_ptr);
        }
        catch (PCLException e1)
        {
			
		}
	viewer = rgbVis(pcl_color_ptr,point_size);
  }
  else if (custom_c)
  {
	io::loadPCDFile<PointXYZ>(filename.c_str(), *basic_cloud_ptr);
    viewer = customColourVis(basic_cloud_ptr,rr,gg,bb,point_size);
  }
  else
  {
	  cout<<"incorrect option selected"<<endl;
  }
 

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
