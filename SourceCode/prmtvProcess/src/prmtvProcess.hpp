#ifndef PRMTVPROCESS_H_
#define PRMTVPROCESS_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <stdio.h>

using namespace pcl;
using namespace std;

void getCentDist(vector<PointCloud<PointXYZRGBA> >);
void getMinDist(PointCloud<PointXYZRGBA>,PointCloud<PointXYZRGBA>);
#endif 
