#ifndef RGBD2FAET_H
#define RGBD2FEAT_H

#include "pcl/point_types.h"
#include <pcl/point_cloud.h>

#include "imgIndex_kdTree_opencv.h"
#include "param.h"

#include <string.h>

typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;

using namespace std;
using namespace cv;
using namespace pcl;

struct POS3D 
{
	float x;
	float y;
	float z;
};
struct COLOR
{
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

class RgbdToFeat{

    public:
	RgbdToFeat(){

	    m_detector_surf = new cv::SurfFeatureDetector(1000);
	    m_descriptor_surf =  new cv::SurfDescriptorExtractor;

	};
	~RgbdToFeat(){};

	void getFeatLocation(vector<cv::KeyPoint> kp, cv::Mat img_depth, vector<POS3D>& pos_3d_dst);
	void getFeats(const char* path, const int port, const double t, const Eigen::Matrix4f trans_gl, const Eigen::Matrix4f trans_lc, 
		vector<cv::Mat> &desc_dst, vector<POS3D> &pos_3d_dst, vector<COLOR> &color_dst, pointcloud_type::Ptr &plane_cloud_1);


	cv::SurfFeatureDetector *			m_detector_surf;
	cv::SurfDescriptorExtractor *		m_descriptor_surf;

};

#endif

