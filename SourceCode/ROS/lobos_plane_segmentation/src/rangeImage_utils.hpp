
#ifndef RANGE_IMAGE_UTILS
#define RANGE_IMAGE_UTILS

#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <opencv2/opencv.hpp>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_nodes.h>

#include <vector>
#include <limits>

using namespace std;
using namespace pcl;
using namespace octree;


boost::shared_ptr<pcl::RangeImage> readRangeImage (std::string ); 
void display_RangeImage (boost::shared_ptr<pcl::RangeImage> );
void replaceNaNs (cv::Mat &image, float value);

float distPointToPlane(cv::Point3f point, cv::Point3f normal);

void createCorlorPointCloud(const std::vector<pcl::PointCloud<PointXYZ> > &cloudList, pcl::PointCloud<pcl::PointXYZRGB> &colorCloud);

// Taques points and normals form the mixure, be careful with notification
void decodePointCloutFromNormalTrick(
		const pcl::PointCloud<pcl::PointXYZINormal>  &normalsTrickCloud,
		pcl::PointCloud<pcl::PointXYZ>& ,
		pcl::PointCloud<pcl::PointXYZ>& 
		);


void writePontcloudList(std::vector<pcl::PointCloud<pcl::PointXYZ> > cloudList);
pcl::PointCloud<pcl::PointXYZRGB> computePlaneExtraction (pcl::RangeImage::Ptr rrangeImage);

template <typename T>
void mergePointClouds (std::vector<pcl::PointCloud<T> > &cloudList) {

	bool changes = true;
	while (changes) {
		
		changes = false;
		// Compare all with all
		for (size_t i = 0; i < cloudList.size(); ++i) {
			//Find closest one

			Eigen::Matrix3f covMain;
			Eigen::Vector4f meanMain;
			computeMeanAndCovarianceMatrix(cloudList[i], covMain, meanMain);
		
			float minDist = std::numeric_limits<float>::infinity();
			int minIdx;
			for (size_t j = 0; j < cloudList.size(); ++j) {
				if (i!=j) {

					Eigen::Matrix3f covLocal;
					Eigen::Vector4f meanLocal;
					computeMeanAndCovarianceMatrix(cloudList[j], covLocal, meanLocal);

					
					float dist = pcl::L2_Norm(meanMain, meanLocal,3);	
					//std::cout << "dist: " << dist << endl;
					if (dist < minDist) {
						minDist = dist;
						minIdx = j;
					}
				}
			}

			// Merge if very similar
			if (minDist < 0.1) {
				pcl::PointCloud<T> tmpCloud;
				cloudList[i] += cloudList[minIdx];
				cloudList.erase(cloudList.begin() + minIdx);

				changes = true;

				//std::cout << std::endl << "Merging" << std::endl;
			}
			
		}

	}
}




// Template function
template <typename T>
void clusterPointCloud (
		boost::shared_ptr<PointCloud<T> > &pc_ptr, 
		std::vector <pcl::PointCloud<T> > &clusters, 
		float resolution=1) {

	typedef typename pcl::octree::OctreePointCloud<T>::LeafNodeIterator my_OctreeIterator;
	typedef typename pcl::octree::OctreePointCloud<T>::LeafNode my_LeafNode;

	pcl::octree::OctreePointCloud <T> normals_ot (resolution); 
	normals_ot.setInputCloud(pc_ptr);
	normals_ot.addPointsFromInputCloud();

	//std::cout << "algo ::: ::::: " << normals_ot.getTreeDepth() << std::endl;
	my_OctreeIterator itL (&normals_ot);
	int cont = 0;
	//Iterate through all the lead nodes of the octree
	
	std::cout << " Starting clustering! " << std::endl;
	while (*itL ) {
		cout << "Iteration: " << cont << endl;	
		cont++;

		vector<int> idxList;
		my_LeafNode *node =  (my_LeafNode*) itL.getCurrentOctreeNode();

		
		node->getContainerPtr()->getPointIndices(idxList);

		pcl::PointCloud<T> cluster;
		for (size_t i = 0; i < idxList.size();  ++i) { //idxList.size();
			int idx = idxList[i];
			T p = (*pc_ptr)[idx];
			
			cluster.push_back(p);	
		}
		clusters.push_back(cluster);
		itL++;
	}
	
	mergePointClouds<T>(clusters);	

}

#endif



