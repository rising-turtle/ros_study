#ifndef PARAM_H
#define PARAM_H

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <fstream>
//#include <Windows.h>
#include <iostream>
#include <string>
#include "Eigen/Core"
using namespace std;

typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;

#define MAX_DATA_SIZE	64000000	//4000*500*32;
#define MAX_FEAT_NUM	2000000		//���������;
#define FEAT_ORB_DIM	32			//ORB����ά��32���ֽ�;
#define NN				1			//knn num;
#define MAX_QUIRY_DIM	1000		//��������������;
#define MAX_IMG_NUM		4000		//��������ͼƬ��Ŀ;
#define THRED			40000		//�б�����������ֵ,200*200;

#define CAMERA			1
#define XTION			2
#define PI				3.14159265
#define MAX_SPEED		2

struct OUTPUT_PARAM{
	int			img_index[2];
	float		confidence[2];
	float		loc_x[2];		//coordinate: positive: right
	float		loc_y[2];		//front
	float		loc_z[2];		//top
	float		angle_roll[2];  //waiting for test
	float		angle_pitch[2];
	float		angle_yaw[2];
	bool		bOK;
	int			loc_type;		//resolve: 0; Marker: 1;
	bool		bFiltered;
//	SYSTEMTIME	loc_time;		//millisecond
	
};
struct INPUT_PARAM{
	cv::Mat		img_gray;
	cv::Mat		img_depth;
	//1 for 2D resolve; 2 for 3D resolve
	int			flag;	
	unsigned long int cap_time;	//input time of image captured, unit: ms
};
struct TRAJ{
	float		loc_x;
	float		loc_y;
	float		loc_z;
	//	SPEED		v_before;	//calculated based on before adjacent pos
	//	SPEED		v_after;	//calculated based on after pos(between the pos and just pushed back trajectory)
	float		v_x_prev;	
	float		v_y_prev;
	float		v_z_prev;
	//float		v_x_aft;
	//float		v_y_aft;
	//float		v_z_aft;
//	SYSTEMTIME	loc_time;	//millisecond
	unsigned long int cap_time;
};


#endif
