#include "prmtvProcess.hpp"

using namespace std;

int main(int argc, char** argv){

  PointCloud<PointXYZRGBA>::Ptr cloudptr1(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr cloudptr2(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr sceneCloud(new PointCloud<PointXYZRGBA>);

  //  io::loadPCDFile<PointXYZRGBA>(argv[1], *cloudptr1);
  //  io::loadPCDFile<PointXYZRGBA>(argv[2], *cloudptr2);

  io::loadPCDFile<PointXYZRGBA>(argv[1], *sceneCloud);

  vector<PointCloud<PointXYZRGBA>::Ptr > *inputVector1 = new vector<PointCloud<PointXYZRGBA>::Ptr >();

  //  vector<PointCloud<PointXYZRGBA>::Ptr > inputVector2;

  // inputVector1.push_back(*cloudptr1);
  //  inputVector1.push_back(*cloudptr2);

  //  inputVector2.push_back(cloudptr1);
  //  inputVector2.push_back(cloudptr2);
  
  //  getCentDist(inputVector1);
  //  float minDist = getMinDist(cloudptr1,cloudptr2);
  
  getObjClusters(sceneCloud, *inputVector1);

  cout<<"Number of clusters:"<<inputVector1->size()<<endl;
  
  visualizeGroup(*inputVector1);
  
  return -1;
}
