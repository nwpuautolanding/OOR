#ifndef PRMTVPROCESS_H_
#define PRMTVPROCESS_H_

/*This should be cleaned, we don't need all this headers*/
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/search/kdtree.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <stdio.h>

using namespace pcl;
using namespace std;

void getCentDist(vector<PointCloud<PointXYZRGBA> >);
float getMinDist(PointCloud<PointXYZRGBA>::Ptr, PointCloud<PointXYZRGBA>::Ptr);
void getGroup(vector<PointCloud<PointXYZRGBA>::Ptr > &clusters, float threshold);

void getObjClusters(PointCloud<PointXYZRGBA>::Ptr sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector);
void getCylClusters(PointCloud<PointXYZRGBA>::Ptr sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector);
void getSphClusters(PointCloud<PointXYZRGBA>::Ptr sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector);
void getPlnClusters(PointCloud<PointXYZRGBA>::Ptr &sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector);

void visualizeGroup(const vector<PointCloud<PointXYZRGBA>::Ptr > &groupClouds);
void improveCloud(PointCloud<PointXYZRGBA>::Ptr &sceneCloud);

#endif 
