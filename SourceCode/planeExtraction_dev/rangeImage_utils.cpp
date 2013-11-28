
#include "rangeImage_utils.hpp"

typedef pcl::PointXYZRGB PointType;

boost::shared_ptr<pcl::RangeImage> readRangeImage (std::string filename) {

        pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
        
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
