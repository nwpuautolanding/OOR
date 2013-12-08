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

using namespace pcl;

boost::shared_ptr<pcl::RangeImage> readRangeImage (std::string filename) {

        pcl::PointCloud<PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<PointXYZRGB>());
        
        pcl::io::loadPCDFile (filename, *point_cloud_ptr);

        // -----------------------------------------------
        // -----Create RangeImage from the PointCloud-----
        // -----------------------------------------------
        float noise_level = 0.0;
        float min_range = 0.0f;
        int border_size = 1;
        float angular_resolution_x = 0.5f;
        float angular_resolution_y = angular_resolution_x;
        angular_resolution_x = pcl::deg2rad (angular_resolution_x);
        angular_resolution_y = pcl::deg2rad (angular_resolution_y);

        Eigen::Affine3f scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud_ptr->sensor_origin_[0],
                                                                     point_cloud_ptr->sensor_origin_[1],
                                                                     point_cloud_ptr->sensor_origin_[2])) *
                                                                Eigen::Affine3f (point_cloud_ptr->sensor_orientation_);
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

        boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
        pcl::RangeImage& range_image = *range_image_ptr;   
        range_image.createFromPointCloud (*point_cloud_ptr, angular_resolution_x, angular_resolution_y,
                                    pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                    scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

        return range_image_ptr;
}


void display_RangeImage (boost::shared_ptr<pcl::RangeImage> range_image_ptr) {

        pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
        range_image_widget.showRangeImage (*range_image_ptr);

        while (!range_image_widget.wasStopped()) {

                range_image_widget.spinOnce();
                pcl_sleep(0.01);
        }
        
}

        
void replaceNaNs (cv::Mat &image, float value) {
        ///typedef cv::Vec3f mt;
        typedef float mt;

        for (cv::MatIterator_<mt> i = image.begin<mt>(); i != image.end<mt>(); ++i)
        {
                if (cvIsNaN(*i)) {
                        *i = value;
                        //cout << "isnan" << endl;

                }
        }

}

// The normal vector must be of length 1
float distPointToPlane (cv::Point3f point, cv::Point3f normal) {

        float sn, sd, sb;
        sn = -normal.dot(point);
        //sd = normal.dot(normal);
        //sb = sn/sb;

        //std::cout << "sn, sd, sb: " << sn << ", " << sd << ", " << sb << endl;
        /*
        sn = -dot( PL.n, (P - PL.V0));
        sd = dot(PL.n, PL.n);
        sb = sn / sd;
        */

        return sn;
}

void createCorlorPointCloud (const std::vector<pcl::PointCloud<PointXYZ> > &cloudList,
		pcl::PointCloud<pcl::PointXYZRGB> &colorCloud) {

	srand (time(NULL));
	for (size_t i = 0; i < cloudList.size(); ++i)
	{
		// Create new color
		unsigned int color [3];
		color[0] = round(rand()%255);
		color[1] = round(rand()%255);
		color[2] = round(rand()%255);
		
		for (size_t j = 0; j < cloudList[i].size(); ++j) { 
			// Create new point with color
			PointXYZRGB finalPoint;
			/*
			finalPoint.x = rangePoint.x;
			finalPoint.y = rangePoint.y;
			finalPoint.z = rangePoint.z;
			*/
			finalPoint.x = cloudList[i][j].x;
			finalPoint.y = cloudList[i][j].y;
			finalPoint.z = cloudList[i][j].z;
			finalPoint.r = color[0];
			finalPoint.g = color[1];
			finalPoint.b = color[2];

			colorCloud.push_back(finalPoint);
			
		}
	}
	std::cout << "size cloudList: " << cloudList.size() << std::endl;
	std::cout << "size colorCloud: " << colorCloud.size() << std::endl;
}

/*
 * XYZ position of the cloud is actually the norm and the norm is actually
 * the XYZ position. Here we want to create a point cloud of the positions
 * encoded in the Norm
 */
void decodePointCloutFromNormalTrick (
		const pcl::PointCloud<pcl::PointXYZINormal>  &normalsTrickCloud,
		pcl::PointCloud<pcl::PointXYZ> &clusteredPoints,
		pcl::PointCloud<pcl::PointXYZ> &clusteredNormals
		) {

	for (size_t j = 0; j < normalsTrickCloud.size(); ++j)
	{
		PointXYZ p;
		p.x = normalsTrickCloud[j].normal_x;
		p.y = normalsTrickCloud[j].normal_y;
		p.z = normalsTrickCloud[j].normal_z;
		
		clusteredPoints.push_back(p);

		p.x = normalsTrickCloud[j].x;
		p.y = normalsTrickCloud[j].y;
		p.z = normalsTrickCloud[j].z;

		clusteredNormals.push_back(p);

	}


}


void writePontcloudList(std::vector<pcl::PointCloud<pcl::PointXYZ> > cloudList) {

	for (size_t i = 0; i < cloudList.size(); ++i)
	{
		ostringstream fileName;
		fileName << "Plane_"<< i << "_" << cloudList[i].size() << ".pcd";
		pcl::io::savePCDFile(fileName.str(), cloudList[i], true);
	}

}


void computePlaneExtraction (pcl::RangeImage::Ptr range_image_ptr) {


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
	
/*
	for (size_t i = 0; i < normal_clusterList.size(); ++i)
	{
		PointCloud <PointXYZ> clusteredPoints;
		PointCloud <PointXYZ> clusteredNormals;
		decodePointCloutFromNormalTrick( normal_clusterList[i], clusteredPoints, clusteredNormals );

		planeList.push_back(clusteredPoints);
		//writePontcloudList(planeList);

	}
*/
	//PointCloud<PointXYZRGB> midSeg;
	//createCorlorPointCloud(planeList, midSeg);
	//io::savePCDFile("mid_plane_pointcloud.pcd", midSeg, true);

	planeList.clear();
	boost::shared_ptr<PointCloud<PointXYZINormal> > dist_PointCloud (new PointCloud<PointXYZINormal>());
	for (size_t i = 0; i < normal_clusterList.size(); ++i)
	{

		Eigen::Matrix3f covMat;
		Eigen::Vector4f meanNorm;
		pcl::computeMeanAndCovarianceMatrix<PointXYZINormal>(normal_clusterList[i], covMat, meanNorm);
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
//	writePontcloudList(planeList);
	createCorlorPointCloud(planeList, finalSeg);

}
