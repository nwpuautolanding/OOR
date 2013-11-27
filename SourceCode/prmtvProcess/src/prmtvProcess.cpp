#include "prmtvProcess.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

using namespace pcl;
using namespace std;

/*Function finds distances between centroids of the input clusters*/

void getCentDist(vector<PointCloud<PointXYZRGBA> > clusterClouds){

  PointCloud<PointXYZRGBA> tempCloud;
  PointCloud<PointXYZRGBA> tempCloud2;
  
  Eigen::Vector4f centroidVec1(0,0,0,0);
  Eigen::Vector4f centroidVec2(0,0,0,0);

  PointXYZ centroid1;
  PointXYZ centroid2;

  for (vector<PointCloud<PointXYZRGBA> >::iterator it = clusterClouds.begin (); it != clusterClouds.end (); ++it) {      
    compute3DCentroid(*it, centroidVec1);
    centroid1.getVector4fMap() = centroidVec1;
      
    for (vector<PointCloud<PointXYZRGBA> >::iterator it2 = clusterClouds.begin (); it2 != clusterClouds.end (); ++it2) {
      compute3DCentroid(*it2, centroidVec2);
      centroid2.getVector4fMap() = centroidVec2;
      float dist = euclideanDistance(centroid1, centroid2);      
      /* 
	 STILL TO BE DECIDED WHAT WE WANT TO RETURN
      */
      cout << "Distance equals:" << dist << endl;
    }
  }
}

/*Function finds minimal distance between two point clouds*/

float getMinDist(PointCloud<PointXYZRGBA>::Ptr cloud1, PointCloud<PointXYZRGBA>::Ptr cloud2){

  KdTreeFLANN<pcl::PointXYZRGBA> cloud1KdTree;
  cloud1KdTree.setInputCloud(cloud1);

  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  vector<float> outputDistances;

  int K = 1;
  bool foundDist = false;

  for(PointCloud<PointXYZRGBA>::iterator it = cloud2->points.begin(); it != cloud2->points.end(); ++it){
    if (cloud1KdTree.nearestKSearch(*it,K , pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
      outputDistances.push_back(*min_element(pointRadiusSquaredDistance.begin(),pointRadiusSquaredDistance.end()));
    }
  }

  return *min_element(outputDistances.begin(),outputDistances.end());
}

void getObjClusters(PointCloud<PointXYZRGBA>::Ptr sceneCloud,vector<PointCloud<PointXYZRGBA>::Ptr > &outVector){

  vector<PointCloud<PointXYZRGBA> > outClusters;
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<PointXYZRGBA>);
  tree->setInputCloud (sceneCloud);

  vector<PointIndices> cluster_indices;
  EuclideanClusterExtraction<PointXYZRGBA> ec;

  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (sceneCloud->points.size());
  ec.setSearchMethod (tree);
  ec.setInputCloud (sceneCloud);
  ec.extract(cluster_indices);

  int j = 0;
  PCDWriter writer;
  
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    PointCloud<PointXYZRGBA>::Ptr cloud_cluster (new PointCloud<PointXYZRGBA>);

    for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (sceneCloud->points[*pit]); //*

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
    stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    //    writer.write<PointXYZRGBA> (ss.str (), *cloud_cluster, false); 
    outClusters.push_back(*cloud_cluster);
    outVector.push_back(cloud_cluster);
    j++;
  }  
}

void getCylClusters(PointCloud<PointXYZRGBA>::Ptr sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector){

  typedef PointXYZRGBA PointT;

  NormalEstimation<PointT, Normal> ne;
  SACSegmentationFromNormals<PointT, Normal> seg; 
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  PointCloud<Normal>::Ptr cloud_normals (new PointCloud<pcl::Normal>);

  ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);
  ExtractIndices<PointT> extract;

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

  // Write the cylinder inliers to disk
  extract.setInputCloud (sceneCloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  PointCloud<PointT>::Ptr cloud_cylinder (new PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  
  outVector.push_back(cloud_cylinder);
}

void getSphClusters(PointCloud<PointXYZRGBA>::Ptr sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector){

  typedef pcl::PointXYZRGBA PointT;

  NormalEstimation<PointT, Normal> ne;
  SACSegmentationFromNormals<PointT, Normal> seg; 
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  PointCloud<pcl::Normal>::Ptr cloud_normals (new PointCloud<Normal>);

  ModelCoefficients::Ptr coefficients_cylinder (new ModelCoefficients);
  PointIndices::Ptr  inliers_cylinder (new PointIndices);
  ExtractIndices<PointT> extract;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (sceneCloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_SPHERE);
  seg.setMethodType (SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (sceneCloud);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  cerr << "Cylinder coefficients: " << *coefficients_cylinder << endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (sceneCloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  PointCloud<PointT>::Ptr cloud_cylinder (new PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);

  //  if(cloud_cylinder->points.size()>20)
  outVector.push_back(cloud_cylinder);
}

/* This functions actually extracts planes from the point cloud */

void getPlnClusters(PointCloud<PointXYZRGBA>::Ptr &sceneCloud, vector<PointCloud<PointXYZRGBA>::Ptr > &outVector){

  typedef pcl::PointXYZRGBA PointT;

  NormalEstimation<PointT, Normal> ne;
  SACSegmentationFromNormals<PointT, Normal> seg; 
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  PointCloud<pcl::Normal>::Ptr cloud_normals (new PointCloud<Normal>);

  ModelCoefficients::Ptr coefficients_cylinder (new ModelCoefficients);
  PointIndices::Ptr  inliers_cylinder (new PointIndices);
  ExtractIndices<PointT> extract;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (sceneCloud);
  ne.setKSearch (15);
  ne.compute (*cloud_normals);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_NORMAL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.01);
  seg.setInputCloud (sceneCloud);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  extract.setInputCloud (sceneCloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (true);
  PointCloud<PointT>::Ptr cloud_cylinder (new PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  
  if(cloud_cylinder->points.size()>500){
    outVector.push_back(cloud_cylinder);
    sceneCloud = cloud_cylinder;
  }
}

/* Function to visualize lines between centroids of group of clouds*/

void visualizeGroup(const vector<PointCloud<PointXYZRGBA>::Ptr > &groupClouds){
  
  Eigen::Vector4f centroidVec1(0,0,0,0);
  Eigen::Vector4f centroidVec2(0,0,0,0);

  PointXYZ centroidPoint1;
  PointXYZ centroidPoint2;

  visualization::PCLVisualizer vis;
  stringstream ss ("element");

  double r = (rand() % 100);
  double g = (rand() % 100);
  double b = (rand() % 100);
  double max_channel = std::max (r, std::max (g, b));

  int i = 0;
  for(vector<PointCloud<PointXYZRGBA>::Ptr >::const_iterator it = groupClouds.begin(); it != groupClouds.end(); ++it){

    ss << i;
    vis.addPointCloud<PointXYZRGBA>(*it, ss.str());

    compute3DCentroid(**it, centroidVec1);
    centroidPoint1.getVector4fMap() = centroidVec1;

    if(it!=groupClouds.end()-1){
      compute3DCentroid(**(it+1), centroidVec2);
      centroidPoint2.getVector4fMap() = centroidVec2;
      ss << i;
      vis.addLine(centroidPoint1, centroidPoint2, r, g, b, ss.str());
    }
    r = (rand() % 100);
    g = (rand() % 100);
    b = (rand() % 100);
    max_channel = std::max (r, std::max (g, b));

    r /= max_channel;
    g /= max_channel;
    b /= max_channel;
    i++;
    vis.spin ();
  }
  //  ss << i;
  //  vis.addPointCloud<PointXYZRGBA>(groupClouds.end(), ss.str());
  vis.spin ();
  //  vis.resetCamera ();
}

void getGroup(vector<PointCloud<PointXYZRGBA>::Ptr > &clusters, float threshold){

}
