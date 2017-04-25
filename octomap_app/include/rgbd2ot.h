#ifndef RGBD2OT_H
#define RGBD2OT_H

#include "ColorOctreeImpl.h"
#include "octomap/FeatOcTree.h"
#include "octomap/FeatOctoMapIO.h"


#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <map>

//opencv
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv/cvwimage.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/eigen.h>
#include <pcl/registration/transforms.h>
#include "tf/tf.h"
#include "tf_conversions/tf_eigen.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>

#define D2R 0.017453 //pi/180
#define R2D 57.296 //180/pi
#define PI 3.1415926

typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;

typedef union
{
	struct /*anonymous*/
	{
		unsigned char Blue;
		unsigned char Green;
		unsigned char Red;
		unsigned char Alpha;
	};
	float float_value;
	long long_value;
} RGBValue;


using namespace std;
using namespace cv;


pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_img, 
		const cv::Mat& rgb_img
		);

bool readPoseData(const char*, std::map<double, vector<double> > &);
void ot2pcd(const ColorOctreeImpl* m_pOctoTree, pointcloud_type& cloud);

void reconstructPcd(const char* path, const int port, const double timestamp, 
	const Eigen::Matrix4f trans,  pointcloud_type::Ptr& pcd);

void loadRgbdIndex(const char* path, std::map<double, string>& index);
void loadAllRgbdIndex(const char* path, const int, const int, vector< std::map<double, string> >& vIndex);

void synPoseToRgbd(std::map<double, vector<double> > pose, vector<std::map<double, string> >vIndex, double minTimeItv, 
	std::map<double, vector<double> >& pose_idx);
void getOtFromSynData(const char* path, 
	const int s_port, const int e_port,
	const vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > vCamToBase,
	std::map<double, vector<double> > pose_idx, ColorOctreeImpl* m_pOctoTree);
void getFeatOtFromSynData(const char* path,  
	const int s_port, const int e_port,
	const vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > vCamToBase,
	std::map<double, vector<double> > pose_idx, FeatOcTree* m_pFeatOctoTree, pointcloud_type::Ptr& m_pFeatPcd);


#endif

