#include "prmtvProcess.hpp"

using namespace pcl;
using namespace std;

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
      cout << "Distance equals:" << dist << endl;
    }
  }
}

void getMinDist(PointCloud<PointXYZRGBA> cloud1,PointCloud<PointXYZRGBA> cloud2){

}




