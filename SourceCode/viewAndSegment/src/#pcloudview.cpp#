// Original code by Geoffrey Biggs, taken from the PCL tutorial in
// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;

PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>); // A cloud that will store colour info.
PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>);    // A fallback cloud with just depth data.
boost::shared_ptr<visualization::CloudViewer> viewer;                 // Point cloud viewer object.
Grabber* kinectGrabber;                                               // OpenNI grabber that takes data from Kinect.
unsigned int filesSaved = 0;                                          // For the numbering of the clouds saved to disk.
bool saveCloud(false), noColour(false);                               // Program control.

void getPlanes(PointCloud<PointXYZ>::Ptr cloud_filtered){
  cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << endl;

  PointCloud<PointXYZ>::Ptr cloud_p (new PointCloud<pcl::PointXYZ>), cloud_f (new PointCloud<PointXYZ>);
  // Write the downsampled version to disk
  ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
  PointIndices::Ptr inliers (new PointIndices ());
  // Create the segmentation object
  SACSegmentation<PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  ExtractIndices<PointXYZ> extract;
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      cerr << "Could not estimate a planar model for the given dataset." << endl;
      break;
    }
    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << endl;
    pcl::PCDWriter writer;
    stringstream ss;
    ss << "plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_p, 0, 255, 0);
    viewer->showCloud(cloud_p, single_color);
    i++;
  }
}

void printUsage(const char* programName)
{
    cout << "Usage: " << programName << " [options]"
         << endl
         << endl
         << "Options:\n"
         << endl
         << "\t<none>     start capturing from a Kinect device.\n"
         << "\t-v NAME    visualize the given .pcd file.\n"
         << "\t-h         shows this help.\n";
}

// This function is called every time the Kinect has new data.
void grabberCallback(const PointCloud<PointXYZ>::ConstPtr& cloud)
{
    if (! viewer->wasStopped())
        viewer->showCloud(cloud);

    if (saveCloud)
	{
		PointCloud<PointXYZ>::Ptr chkThis(new PointCloud<pcl::PointXYZ>());
		copyPointCloud<PointXYZ, PointXYZ>(*cloud, *chkThis);


		getPlanes(chkThis);

        saveCloud = false;
    }
}
// For detecting when SPACE is pressed.
void keyboardEventOccurred(const visualization::KeyboardEvent& event,
    void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        saveCloud = true;
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer>
createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v
        (new visualization::CloudViewer("3D Viewer"));
    v->registerKeyboardCallback(keyboardEventOccurred);

    return(v);
}

int
main(int argc, char** argv)
{
    if (console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return 0;
    }

    bool justVisualize(false);
    string filename;
    if (console::find_argument(argc, argv, "-v") >= 0)
    {
        if (argc != 3)
        {
            printUsage(argv[0]);
            return 0;
        }

        filename = argv[2];
        justVisualize = true;
    }
    else if (argc != 1)
    {
        printUsage(argv[0]);
        return 0;
    }

    // First mode, open and show a cloud from disk.
    if (justVisualize)
    {
        // Try with colour information...
        try
        {
            io::loadPCDFile<PointXYZRGBA>(filename.c_str(), *cloudptr);
        }
        catch (PCLException e1)
        {
            try
            {
                io::loadPCDFile<PointXYZ>(filename.c_str(), *fallbackCloud);
            }
            catch (PCLException e2)
            {
                return -1;
            }

            noColour = true;
        }

        cout << "Loaded " << filename << "." << endl;
        if (noColour)
            cout << "This file has no RGBA colour information present." << endl;
    }
    // Second mode, start fetching and displaying frames from Kinect.
    else
    {
        kinectGrabber = new OpenNIGrabber();
        if (kinectGrabber == 0)
            return false;
        boost::function<void (const PointCloud<PointXYZ>::ConstPtr&)> f =
            boost::bind(&grabberCallback, _1);
        kinectGrabber->registerCallback(f);
    }

    viewer = createViewer();

    if (justVisualize)
    {
        if (noColour)
            viewer->showCloud(fallbackCloud);
        else viewer->showCloud(cloudptr);
    }
    else kinectGrabber->start();

    // Main loop.
    while (! viewer->wasStopped())
        boost::this_thread::sleep(boost::posix_time::seconds(1));

    if (! justVisualize)
        kinectGrabber->stop();
}
