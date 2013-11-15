#include "prmtvProcess.hpp"

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

void getMinDist(PointCloud<PointXYZRGBA>::Ptr cloud1,PointCloud<PointXYZRGBA>::Ptr cloud2){

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

  if(!outputDistances.empty()){
    cout<<"Minimum distance is:"<<*min_element(outputDistances.begin(),outputDistances.end())<<endl;
    cout<<"Maximum distance is:"<<*max_element(outputDistances.begin(),outputDistances.end())<<endl;
  }
}




