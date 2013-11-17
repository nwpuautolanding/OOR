#include "prmtvProcess.hpp"

using namespace std;

int main(int argc, char** argv){

  PointCloud<PointXYZRGBA>::Ptr cloudptr1(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr cloudptr2(new PointCloud<PointXYZRGBA>);

  io::loadPCDFile<PointXYZRGBA>(argv[1], *cloudptr1);
  io::loadPCDFile<PointXYZRGBA>(argv[2], *cloudptr2);

  vector<PointCloud<PointXYZRGBA> > inputVector1;
  vector<PointCloud<PointXYZRGBA>::Ptr > inputVector2;

  inputVector1.push_back(*cloudptr1);
  inputVector1.push_back(*cloudptr2);

  inputVector2.push_back(cloudptr1);
  inputVector2.push_back(cloudptr2);
  
  //  getCentDist(inputVector1);
  //  float minDist = getMinDist(cloudptr1,cloudptr2);

  visualizeGroup(inputVector2);
  
  return -1;
}
