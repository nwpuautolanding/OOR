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
#include <pcl/features/normal_3d.h>
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
//void getCylClusters(PointCloud<PointXYZRGBA>::Ptr sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector);
void getSphClusters(PointCloud<PointXYZRGBA>::Ptr sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector);
void getPlnClusters(PointCloud<PointXYZRGBA>::Ptr &sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector);

void visualizeGroup(const vector<PointCloud<PointXYZRGBA>::Ptr > &groupClouds);
void improveCloud(PointCloud<PointXYZRGBA>::Ptr &sceneCloud);

template <typename T>
void getCylClusters (boost::shared_ptr<PointCloud<T> > sceneCloud, vector<boost::shared_ptr<PointCloud<T> > > &outVector) {

  typedef typename pcl::search::KdTree<T>::Ptr my_KdTreePtr;
  typedef typename pcl::PointCloud<T>::Ptr my_PointCloudPtr;

  NormalEstimation<T, Normal> ne;
  SACSegmentationFromNormals<T, Normal> seg; 
  my_KdTreePtr tree (new pcl::search::KdTree<T> ());
  PointCloud<Normal>::Ptr cloud_normals (new PointCloud<pcl::Normal>);

  ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);
  ExtractIndices<T> extract;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (sceneCloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_CYLINDER);
  seg.setMethodType (SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (sceneCloud);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  cout << " Number of inliers vs total number of points " << inliers_cylinder->indices.size() << " vs " << sceneCloud->size() << endl; 
  // Write the cylinder inliers to disk
  extract.setInputCloud (sceneCloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  my_PointCloudPtr cloud_cylinder (new PointCloud<T> ());
  extract.filter (*cloud_cylinder);
  
  outVector.push_back(cloud_cylinder);
}

#endif 
