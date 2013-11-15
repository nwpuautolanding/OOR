#include "prmtvProcess.hpp"

using namespace std;

int main(int argc, char** argv){

  PointCloud<PointXYZRGBA>::Ptr cloudptr1(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr cloudptr2(new PointCloud<PointXYZRGBA>);
  
  io::loadPCDFile<PointXYZRGBA>(argv[1], *cloudptr1);
  io::loadPCDFile<PointXYZRGBA>(argv[2], *cloudptr2);

  vector<PointCloud<PointXYZRGBA> > inputVector;

  inputVector.push_back(*cloudptr1);
  inputVector.push_back(*cloudptr2);

  getCentDist(inputVector);

  return -1;
}
