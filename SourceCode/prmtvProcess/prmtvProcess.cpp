#include "prmtvProcess.hpp"

using namespace pcl;
using namespace std;

vector< PointCloud<PointXYZRGBA>::Ptr> groupPrmtvs(vector< PointCloud<PointXYZ>::Ptr > clusterClouds){
  
    for (vector< PointCloud<PointXYZ>::Ptr>::const_iterator it = clusterClouds.begin (); it != clusterClouds.end (); ++it) {
      
      Eigen::Vector3f clusterCentroid = it->getCentroid();

      for (vector< PointCloud<PointXYZ>::Ptr>::const_iterator it2 = clusterClouds.begin (); it2 != clusterClouds.end (); ++it2) {
	Eigen::Vector3f tempCentroid = it2->getCentroid();
	float dist = euclideanDistance(&clusterCentroid, &tempCentroid);
      }
  }
 PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>);
}

void main(int argc, char** argv){

  vector< PointCloud<PointXYZRGBA>::Ptr> inputGroup;
  PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr cloudptr2(new PointCloud<PointXYZRGBA>);

  io::loadPCDFile<PointXYZRGBA>(argv[0], *cloudptr);
  io::loadPCDFile<PointXYZRGBA>(argv[0], *cloudptr2);

  inputGroup.push_back(cloudptr);
  inputGroup.push_back(cloudptr2);
}
