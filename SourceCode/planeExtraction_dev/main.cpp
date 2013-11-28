#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_nodes.h>
#include "rangeImage_utils.hpp"

using namespace std;
using namespace pcl;
using namespace octree;


int main (int argn, char ** argv) {

	std::string filename = std::string(argv[1]);

	boost::shared_ptr<pcl::RangeImage> range_image_ptr;
    	range_image_ptr = readRangeImage (filename);

	
	// Create horitzontal and vertical vectors	
	cv::Mat *hori = new cv::Mat(range_image_ptr->height, range_image_ptr->width, CV_32FC3);
	cv::Mat *vert = new cv::Mat(range_image_ptr->height, range_image_ptr->width, CV_32FC3);
	for (int i = 1; i < range_image_ptr->width-1; ++i) {
		for (int j = 1; j < range_image_ptr->height-1; ++j) {
		
			hori->at<cv::Vec3f>(j,i)[0] = range_image_ptr->getPoint(i-1,j).x - range_image_ptr->getPoint(i+1,j).x;
			hori->at<cv::Vec3f>(j,i)[1] = range_image_ptr->getPoint(i-1,j).y - range_image_ptr->getPoint(i+1,j).y;
			hori->at<cv::Vec3f>(j,i)[2] = range_image_ptr->getPoint(i-1,j).z - range_image_ptr->getPoint(i+1,j).z;

			vert->at<cv::Vec3f>(j,i)[0] = range_image_ptr->getPoint(i,j-1).x - range_image_ptr->getPoint(i,j+1).x;
			vert->at<cv::Vec3f>(j,i)[1] = range_image_ptr->getPoint(i,j-1).y - range_image_ptr->getPoint(i,j+1).y;
			vert->at<cv::Vec3f>(j,i)[2] = range_image_ptr->getPoint(i,j-1).z - range_image_ptr->getPoint(i,j+1).z;
			
		}
	}
	
	// Clean results
	//replaceNaNs(*hori, 50);
	//replaceNaNs(*vert, 50);
	cv::GaussianBlur(*hori, *hori, cv::Size(3,3), 0);
	cv::GaussianBlur(*vert, *vert, cv::Size(3,3), 0);

	//Compute normals	
	cv::Mat *normals = new cv::Mat(hori->size(), CV_32FC3);
	for (int i = 0; i < normals->cols; ++i) {
		for (int j = 0; j < normals->rows; ++j) {
			//std::cout << normals.at<cv::Vec3f>(i,j) << std::endl;
			cv::Vec3f normalizedNorm;
			cv::Vec3f crossVector;

			crossVector = hori->at<cv::Vec3f>(j,i).cross(vert->at<cv::Vec3f>(j,i));
			cv::normalize(crossVector, normalizedNorm);
			
			//cout << "hori and vert: " << hori->at<cv::Vec3f>(j,i) << " --- " << vert->at<cv::Vec3f>(j,i) << endl;
			//cout << "Cross and normalized: " << crossVector << " --- " << normalizedNorm << endl;
			normals->at<cv::Vec3f>(j,i) = normalizedNorm; 
			//std::cout << normals.at<cv::Vec3f>(i,j) << std::endl;
		}
	}


	// Create point cloud where the normal is the position and the pixel index is in the normal
	boost::shared_ptr<PointCloud <PointXYZINormal> > normals_pc_ptr (new PointCloud<PointXYZINormal> ());
	for (int i = 0; i < normals->cols; ++i) { //normals.cols
		for (int j = 0; j < normals->rows; ++j) {
			pcl::PointXYZINormal p;
			cv::Vec3f n = normals->at<cv::Vec3f>(j,i);

			
			p.x = n[0];
			p.y = n[1];
			p.z = n[2];
		
			/*
			p.normal_x = j;
			p.normal_y = i;
			*/
			//if (cvIsNaN(range_image_ptr->at(i,j).x)) {
				//p.normal_x = 50;
				//p.normal_y = i;
				//p.normal_z = j;
				//cout << "NAN!!!" << endl;
			//} else { 
				p.normal_x = range_image_ptr->at(i, j).x;
				p.normal_y = range_image_ptr->at(i, j).y;
				p.normal_z = range_image_ptr->at(i, j).z;
			//}	
			normals_pc_ptr->push_back(p);
		}
	}


	// Clusterize normals with octree
	vector <PointCloud <PointXYZ> > planeList;
	vector <PointCloud <PointXYZINormal> > normal_clusterList;
	clusterPointCloud <PointXYZINormal> (normals_pc_ptr, normal_clusterList);
	
	for (size_t i = 0; i < normal_clusterList.size(); ++i)
	{
		PointCloud <PointXYZ> clusteredPoints;
		PointCloud <PointXYZ> clusteredNormals;
		decodePointCloutFromNormalTrick( normal_clusterList[i], clusteredPoints, clusteredNormals );

		planeList.push_back(clusteredPoints);
		//writePontcloudList(planeList);

		/*
		Eigen::Matrix3f covariance_matrix;
		Eigen::Vector4f xyz_centroid;
		computeMeanAndCovarianceMatrix(clusteredNormals, covariance_matrix, xyz_centroid);
		
		cout << "centroid: " << endl << xyz_centroid << endl;
		cout << "covariance: " << endl << covariance_matrix << endl << endl;
		*/
	}

	PointCloud<PointXYZRGB> midSeg;
	createCorlorPointCloud(planeList, midSeg);
	io::savePCDFile("mid_plane_pointcloud.pcd", midSeg, true);

	planeList.clear();
	boost::shared_ptr<PointCloud<PointXYZINormal> > dist_PointCloud (new PointCloud<PointXYZINormal>());
	for (size_t i = 0; i < normal_clusterList.size(); ++i)
	{

		Eigen::Matrix3f covMat;
		Eigen::Vector4f meanNorm;
		computeMeanAndCovarianceMatrix(normal_clusterList[i], covMat, meanNorm);
		dist_PointCloud->clear();
		for (size_t j = 0; j < normal_clusterList[i].size(); ++j)
		{
			PointXYZINormal p;

			cv::Point3f normal, point;
			
			//Copy normal and point, not intuitive
			normal.x = meanNorm[0];
			normal.y = meanNorm[1];
			normal.z = meanNorm[2];
			// Copy actual point
			point.x = normal_clusterList[i][j].normal_x;
			point.y = normal_clusterList[i][j].normal_y;
			point.z = normal_clusterList[i][j].normal_z;

			p.x = distPointToPlane(point, normal);
			p.y = 0;
			p.z = 0;
			p.normal_x = normal_clusterList[i][j].normal_x;
			p.normal_y = normal_clusterList[i][j].normal_y;
			p.normal_z = normal_clusterList[i][j].normal_z;
			//cout << "Distance ------- " << p.x << endl;

			dist_PointCloud->push_back(p);

		}

		//cout << "dist_PointCloud: " << dist_PointCloud->size() << endl;
		vector <PointCloud<PointXYZINormal> > clusterResult;
		clusterPointCloud<PointXYZINormal>(dist_PointCloud, clusterResult, 0.001);
		//cout << "clusterResult size: " << clusterResult.size() << endl;
		for (size_t j = 0; j < clusterResult.size(); ++j)
		{
			PointCloud<PointXYZ> plane;
			PointCloud<PointXYZ> normals;
			decodePointCloutFromNormalTrick(clusterResult[j], plane, normals);
			planeList.push_back(plane);

		}
		
	}


	//finalSeg.clear();

	PointCloud<PointXYZRGB> finalSeg;
	writePontcloudList(planeList);
	createCorlorPointCloud(planeList, finalSeg);
	cout << "final size: " << planeList.size() << endl;

	//io::savePCDFile("plane_pointcloud.pcd", testRange, true);
	io::savePCDFile("plane_pointcloud.pcd", finalSeg, true);
	return 0;
}
