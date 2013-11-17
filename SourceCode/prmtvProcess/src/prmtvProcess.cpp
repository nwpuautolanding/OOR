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

float getMinDist(PointCloud<PointXYZRGBA>::Ptr cloud1,PointCloud<PointXYZRGBA>::Ptr cloud2){

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

void visualizeGroup(const vector<PointCloud<PointXYZRGBA>::Ptr > groupClouds){
  
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

    compute3DCentroid(**(it+1), centroidVec2);
    centroidPoint2.getVector4fMap() = centroidVec2;
    
    ss << i;
    vis.addLine(centroidPoint1, centroidPoint2, r, g, b, ss.str());

    r = (rand() % 100);
    g = (rand() % 100);
    b = (rand() % 100);
    max_channel = std::max (r, std::max (g, b));

    r /= max_channel;
    g /= max_channel;
    b /= max_channel;
    i++;
  }

  vis.resetCamera ();
  vis.spin ();
}
