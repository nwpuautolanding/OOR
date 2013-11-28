#include "prmtvProcess.hpp"

#include <pcl/surface/bilateral_upsampling.h>


using namespace std;

int main(int argc, char** argv){

  PointCloud<PointXYZRGBA>::Ptr cloudptr1(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr cloudptr2(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr sceneCloud(new PointCloud<PointXYZRGBA>);

  //  io::loadPCDFile<PointXYZRGBA>(argv[1], *cloudptr1);
  //  io::loadPCDFile<PointXYZRGBA>(argv[2], *cloudptr2);

  io::loadPCDFile<PointXYZRGBA>(argv[1], *sceneCloud);

  vector<PointCloud<PointXYZRGBA>::Ptr > *inputVector1 = new vector<PointCloud<PointXYZRGBA>::Ptr >();
  vector<PointCloud<PointXYZRGBA>::Ptr > *inputVector2 = new vector<PointCloud<PointXYZRGBA>::Ptr >();
  vector<PointCloud<PointXYZRGBA>::Ptr > *inputVector3 = new vector<PointCloud<PointXYZRGBA>::Ptr >();

  //  float minDist = getMinDist(cloudptr1,cloudptr2);
  
  //  improveCloud(sceneCloud);

  int prevSize = inputVector2->size();
  int newSize = inputVector2->size()+1;
  
  while(prevSize<newSize){
    prevSize = inputVector2->size();
    getPlnClusters(sceneCloud, *inputVector2);
    newSize = inputVector2->size();
    }
  
  cout<<"Number of clusters:"<<inputVector2->size()<<endl;

  getObjClusters(sceneCloud, *inputVector1);
  
  
  //  getCentDist(*inputVector1);  
  
  //  cout<< getMinDist(inputVector1->at(0),inputVector1->at(1))<< endl;

  //  inputVector1->erase(inputVector1->begin()+1,inputVector1->begin()+3);

  //    for(size_t i = 0; i<inputVector1->size();i++)
  //   getCylClusters(inputVector1->at(i), *inputVector3);  
  
  /*PCDWriter writer;
  writer.write<PointXYZRGBA> ("sceneCloud.pcd", *sceneCloud, false); 
  */  
  improveCloud(sceneCloud);
  inputVector1->push_back(sceneCloud);
  
  visualizeGroup(*inputVector1);

  return -1;
}

