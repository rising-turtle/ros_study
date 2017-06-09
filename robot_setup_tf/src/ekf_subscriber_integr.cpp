//location of robot_pose_ekf
//cd /opt/ros/jade/share/robot_pose_ekf/
//robot_pose_ekf topic /robot_pose_ekf/odom_combined

#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "std_msgs/Int32.h"
//#include "std_msgs/Bool.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
//#include <cv_bridge/cv_bridge.h>

#include "sr4000handler.cpp"
#include "ransac_stat.h"
#include "cam_model.cpp"

//#include <opencv2/opencv.hpp> //included in cam_model.h
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"

#include <iostream>
#include <fstream>
#include <pthread.h>

#include <tf/LinearMath/Matrix3x3.h>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
//#include "pcl/visualization/pcl_visualizer.h"

#include "engine.h" //for matlab
#include "matrix.h" //for matlab

typedef pcl::PointXYZ point_type; 
typedef pcl::PointCloud<point_type> cloud_type; 
typedef typename cloud_type::Ptr  cloudPtr; 

using namespace ros;
using namespace cv;
using namespace std;

cloudPtr g_point_cloud(new cloud_type(176, 144));

/*static const double time_end     = 1264198883.0;
static const double ekf_duration   = 62.0;
static const double EPS_trans_x    = 0.02;
static const double EPS_trans_y    = 0.04;
static const double EPS_trans_z    = 0.00001;
static const double EPS_rot_x      = 0.005;
static const double EPS_rot_y      = 0.005;
static const double EPS_rot_z      = 0.005;
static const double EPS_rot_w      = 0.005;
*/

bool g_cloud_is_ready = false;
bool quat_is_ready = false;
bool vo_is_ready = false;
bool quit = false;

pthread_mutex_t vo_mut_;
pthread_mutex_t g_mut_;
pthread_mutex_t quat_mut_;
pthread_mutex_t quit_mut_;
vector<unsigned char> g_buf_;
geometry_msgs::Quaternion quat_;

bool g_exit = false; 
bool g_syn_flag = true;//false;
bool g_get_msg = false;
int g_count = 0;
int imu_count = 0;
int obs_no=-1;
int proc_obs_no=0;
int state=0;	

Engine *ep;
std::vector<cv::Mat> *stacked_vo_cam; //for saving 7x1 pose trace
std::vector<tf::Vector3> *stacked_rpy_cam; //for saving rpy from cam
std::vector<tf::Vector3> *stacked_rpy_imu; //for saving rpy from cam
//std::vector<ros::Time>  stacked_vo_stamps;
//ros::Time cam_stamp;

std::vector<geometry_msgs::Quaternion> *stacked_vo_imu; //for saving 7x1 pose trace
std::vector<ros::Time>  stacked_imu_stamps;
ros::Time imu_stamp;

std::vector<cv::Mat> xs;
std::vector<cv::Mat> ys;
std::vector<cv::Mat> zs;
std::vector<cv::Mat> I_imgs;
//std::vector<cv::Mat> D_imgs;

inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}
//*********************************************************************************************************
void DISPMAT(cv::Mat a)
{
	cout<<"\n";	
	for (int i=0;i<a.rows;i++)
	{
		std::string tmp;
		for (int j=0;j<a.cols;j++)
		{
			tmp=tmp+std::to_string((long double)a.at<float>(i,j));
			tmp=tmp+" ";
		}
		cout<<tmp<<"\n";
	}
}
//*********************************************************************************************************
cv::Mat r2q(cv::Mat R) //TBD: convert a q to R and then back to q. see if its same
{
	float r11=R.at<float>(0,0);
	float r12=R.at<float>(0,1);
	float r13=R.at<float>(0,2);
	float r21=R.at<float>(1,0);
	float r22=R.at<float>(1,1);
	float r23=R.at<float>(1,2);
	float r31=R.at<float>(2,0);
	float r32=R.at<float>(2,1);
	float r33=R.at<float>(2,2);

	float q0 = ( r11 + r22 + r33 + 1.0f) / 4.0f;
	float q1 = ( r11 - r22 - r33 + 1.0f) / 4.0f;
	float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;

	if(q0 < 0.0f) q0 = 0.0f;
	if(q1 < 0.0f) q1 = 0.0f;
	if(q2 < 0.0f) q2 = 0.0f;
	if(q3 < 0.0f) q3 = 0.0f;
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
	q0 *= +1.0f;
	q1 *= SIGN(r32 - r23);
	q2 *= SIGN(r13 - r31);
	q3 *= SIGN(r21 - r12);
	} else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
	q0 *= SIGN(r32 - r23);
	q1 *= +1.0f;
	q2 *= SIGN(r21 + r12);
	q3 *= SIGN(r13 + r31);
	} else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
	q0 *= SIGN(r13 - r31);
	q1 *= SIGN(r21 + r12);
	q2 *= +1.0f;
	q3 *= SIGN(r32 + r23);
	} else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
	q0 *= SIGN(r21 - r12);
	q1 *= SIGN(r31 + r13);
	q2 *= SIGN(r32 + r23);
	q3 *= +1.0f;
	} else {
	printf("coding error\n");
	}
	
	float r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;

	cv::Mat q=Mat::zeros(4,1,CV_32FC1);

	q.at<float>(0,0)=q0;
	q.at<float>(1,0)=q1;
	q.at<float>(2,0)=q2;
	q.at<float>(3,0)=q3;

	return q;
}
//*********************************************************************************************************
cv::Mat q2r(Mat q)
{
	Mat R=Mat::eye(3,3,CV_32FC1);
	//TBD: See if this function produces good results
	float x=q.at<float>(1,0);
	float y=q.at<float>(2,0);
	float z=q.at<float>(3,0);
	float r=q.at<float>(0,0);

	R.at<float>(0,0)=r*r+x*x-y*y-z*z;
	R.at<float>(0,1)=2*(x*y-r*z);
	R.at<float>(0,2)=2*(z*x+r*y);

	R.at<float>(1,0)=2*(x*y+r*z);
	R.at<float>(1,1)=r*r-x*x+y*y-z*z;
	R.at<float>(1,2)=2*(y*z-r*x);

	R.at<float>(2,0)=2*(z*x-r*y);
	R.at<float>(2,1)=2*(y*z+r*x);
	R.at<float>(2,2)=r*r-x*x-y*y+z*z;

	return R;
}
//*******************************************************************************************************************
void save_stacked_rpy(std::vector<tf::Vector3> stacked_rpy, std::string path)
{
	ofstream myfile;
	myfile.open (path);

	for(int i=0;i<stacked_rpy.size();i++)
	{
		tf::Vector3 elem=stacked_rpy.at(i);
		myfile<<elem[0];
		myfile<<",";
		myfile<<elem[1];
		myfile<<",";
		myfile<<elem[2];
		myfile<<"\n";
	}
	myfile.close();
}
//*******************************************************************************************************************
void save_stacked_vo(std::vector<cv::Mat> stacked_vo, std::string path)
{
	ofstream myfile;
	myfile.open (path);
	for(int i=0;i<stacked_vo.size();i++)
	{
		Mat elem=stacked_vo.at(i);

		for (int j=0;j<elem.rows;j++)
		{
			myfile<<elem.at<float>(j,0);
			if (j<elem.rows-1)
				myfile<<",";
		}
		myfile<<"\n";
	}
	
	myfile.close();
}
//*******************************************************************************************************************
void update_rpy(std::vector<tf::Vector3> *stacked_rpy, double roll, double pitch,double yaw)
{
	/*
	cout<<"\nInput rot:";
	DISPMAT(rot);
	cout<<"\nInput trans:";
	DISPMAT(trans);
	cout<<"\ntransformation matrix:";
	DISPMAT(transf_mat);
	*/
	tf::Vector3 rpy(0,0,0);
	rpy[0]=roll; 
	rpy[1]=pitch; 
	rpy[2]=yaw;
	stacked_rpy->push_back(rpy);			
	//cout<<"New Pose:";
	//DISPMAT(final_op);
}
//*******************************************************************************************************************
void update_vo(std::vector<cv::Mat> *vo,cv::Mat rot,cv::Mat trans)
{
	//LOGD<<"rot size:"<<rot.rows<<"x"<<rot.cols;
	//LOGD<<"trans size:"<<trans.rows<<"x"<<trans.cols;
	Mat transf_mat=Mat::eye(4,4,CV_32FC1);
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			transf_mat.at<float>(i,j)=rot.at<float>(i,j); //3x3 patch update

	for(int i=0;i<3;i++)
		transf_mat.at<float>(i,3)=trans.at<float>(i,0);

	/*
	cout<<"\nInput rot:";
	DISPMAT(rot);
	cout<<"\nInput trans:";
	DISPMAT(trans);
	cout<<"\ntransformation matrix:";
	DISPMAT(transf_mat);
	*/

	int last_idx;
	Mat pos=Mat::ones(4,1,CV_32FC1);
	pos.at<float>(3,0)=1.0;
	Mat last_pose;
	if (vo->size()==0)
	{
		last_pose=Mat::zeros(7,1,CV_32FC1);
		last_pose.at<float>(3,0)=1.0;
	}else
	{
		last_idx=vo->size()-1;
		last_pose=vo->at(last_idx);
	} 

	for(int i=0;i<3;i++)
		pos.at<float>(i,0)=last_pose.at<float>(i,0);
	
	//cout<<"Last Position:";
	//DISPMAT(pos);
	Mat new_pos=transf_mat*pos;
	Mat new_rot=q2r(last_pose(Range(3,7),Range(0,1)))*rot;
	//cout<<"New Position:";
	//DISPMAT(new_pos);
	//cout<<"New rot:";
	//DISPMAT(new_rot);

	Mat final_op;
	vconcat(new_pos(Range(0,3),Range(0,1)),r2q(new_rot),final_op);
	vo->push_back(final_op);
			
	//cout<<"New Pose:";
	//DISPMAT(final_op);
}
//**************************************************************************************************************************
ransac_stat vodometery_matlab(cv::Mat x1, cv::Mat y1, cv::Mat z1, cv::Mat x2, cv::Mat y2, cv::Mat z2, cv::Mat img1, cv::Mat img2, int &state, Engine *ep, int loc_obs_no, cv::Mat imu_rot)
{
	ransac_stat rs_obj; //create empty rs_obj
	////////////////////// FORMAT FILE NAMES ////////////////////////////
	/*
	char buff[200];
	for (int i=0;i<200;i++)
		buff[i]=0;
	string folder_name="./RANSAC_pose_shift_dr_Ye/";
	sprintf(buff,"%sRANSAC_RESULT_%d_%d.yml",folder_name.c_str(),loc_obs_no-1,loc_obs_no);
	string Dr_Ye_File(buff);

	if (fpExists(Dr_Ye_File))
	{
		if (rs_obj.status == 99)
		{
			//LOGD<<"vodometery will be loaded from file";
			//LOGD<<"load_RANSAC_content started";
			//load_RANSAC_content(rs_obj,Dr_Ye_File); // The file this function reads was written during last program execution //TBD
			//LOGD<<"load_RANSAC_content completed";
		}
		return rs_obj;
	}
	*/
	////////////////////// END FORMAT FILE NAMES ////////////////////////	
	int psn=0;
	int SolutionState=-1;

	//Define outputs
	mxArray *rotMat_m=NULL;
	mxArray *transMat_m=NULL;
	mxArray *psn_m=NULL;
	mxArray *pset1_m=NULL;
	mxArray *pset2_m=NULL;
	mxArray *GoodFrames1_m=NULL;
	mxArray *GoodFrames2_m=NULL;
	mxArray *GoodDescriptor1_m=NULL;
	mxArray *GoodDescriptor2_m=NULL;
	mxArray *SolutionState_m=NULL;

	//Define inputs
	mxArray *x1_m=mxCreateDoubleMatrix(144, 176, mxREAL);
	mxArray *x2_m=mxCreateDoubleMatrix(144, 176, mxREAL);
	mxArray *y1_m=mxCreateDoubleMatrix(144, 176, mxREAL);
	mxArray *y2_m=mxCreateDoubleMatrix(144, 176, mxREAL);
	mxArray *z1_m=mxCreateDoubleMatrix(144, 176, mxREAL);
	mxArray *z2_m=mxCreateDoubleMatrix(144, 176, mxREAL);

	mxArray *img1_m=mxCreateDoubleMatrix(144, 176, mxREAL);
	mxArray *img2_m=mxCreateDoubleMatrix(144, 176, mxREAL);

	mxArray *imurot_m=mxCreateDoubleMatrix(3, 3, mxREAL);

	double  *x1_ptr;	/* pointer to real data in mxarray */
	double  *x2_ptr;	/* pointer to real data in mxarray */
	double  *y1_ptr;	/* pointer to real data in mxarray */
	double  *y2_ptr;	/* pointer to real data in mxarray */
	double  *z1_ptr;	/* pointer to real data in mxarray */
	double  *z2_ptr;	/* pointer to real data in mxarray */
	double  *img1_ptr;	/* pointer to real data in mxarray */
	double  *img2_ptr;	/* pointer to real data in mxarray */
	double  *imurot_ptr;	/* pointer to real data in mxarray */

	x1_ptr = mxGetPr(x1_m);
	x2_ptr = mxGetPr(x2_m);
	y1_ptr = mxGetPr(y1_m);
	y2_ptr = mxGetPr(y2_m);
	z1_ptr = mxGetPr(z1_m);
	z2_ptr = mxGetPr(z2_m);
	img1_ptr = mxGetPr(img1_m);
	img2_ptr = mxGetPr(img2_m);
	imurot_ptr = mxGetPr(imurot_m);

	/* Copy data into the mxArray */
	int index=0;
	for (int j=0; j < 176; j++ )
		for (int i=0; i < 144; i++ ) 
		{
			x1_ptr[index] = x1.at<float>(i,j); //Order is correct: confirmed
			x2_ptr[index] = x2.at<float>(i,j); //Order is correct: confirmed
			y1_ptr[index] = y1.at<float>(i,j); //Order is correct: confirmed
			y2_ptr[index] = y2.at<float>(i,j); //Order is correct: confirmed
			z1_ptr[index] = z1.at<float>(i,j); //Order is correct: confirmed
			z2_ptr[index] = z2.at<float>(i,j); //Order is correct: confirmed
			index++;
		}

	index=0;
	//ROS_INFO("ekf_subscriber.cpp: before loop. img1.rows: %d",img1.rows);
	//ROS_INFO("ekf_subscriber.cpp: before loop. img1.cols: %d",img1.cols);
	for (int j=0; j < 176; j++ )
		for (int i=0; i < 144; i++ ) 
		{
			//cout<<"<****\n";
			//cout<<img1.at<unsigned short>(i,j)<<"\n";
			img1_ptr[index] = (double)img1.at<unsigned char>(i,j); //Order is correct: confirmed
			img2_ptr[index] = (double)img2.at<unsigned char>(i,j); //Order is correct: confirmed
			//cout<<img1_ptr[index]<<"\n";
			//cout<<"****>\n";
			index++;
		}
	
	index=0;
	for (int j=0; j < 3; j++)
		for (int i=0; i < 3; i++)
		{
			imurot_ptr[index] = imu_rot.at<float>(i,j);
			index++;
		}

	
	//ROS_INFO("ekf_subscriber.cpp: after loop");
	double *rot_real, *trans_real, *pset1_real, *pset2_real, *psn_real;
	double *GoodFrames1_real, *GoodFrames2_real;
	double *GoodDescriptor1_real, *GoodDescriptor2_real;
	double *SolutionState_real;
	engEvalString(ep,"cd('/home/emaad22/Desktop/forth_height_EKF/code_from_dr_ye')");
	engPutVariable(ep, "x1_m", x1_m); 
	engPutVariable(ep, "x2_m", x2_m); 
	engPutVariable(ep, "y1_m", y1_m); 
	engPutVariable(ep, "y2_m", y2_m); 
	engPutVariable(ep, "z1_m", z1_m); 
	engPutVariable(ep, "z2_m", z2_m);
	engPutVariable(ep, "img1_m", img1_m); 
	engPutVariable(ep, "img2_m", img2_m); 
	engPutVariable(ep, "imurot_m", imurot_m); 

	engEvalString(ep,"[rotMat_m, transMat_m, psn_m, pset1_m, pset2_m, GoodFrames1_m, GoodFrames2_m,  GoodDescriptor1_m, GoodDescriptor2_m, SolutionState_m] = vodometry_imumod(x1_m,x2_m,y1_m,y2_m,z1_m,z2_m,img1_m,img2_m,imurot_m)");

	rotMat_m = engGetVariable(ep,"rotMat_m");
	transMat_m = engGetVariable(ep,"transMat_m");
	psn_m = engGetVariable(ep,"psn_m"); 
	pset1_m = engGetVariable(ep,"pset1_m");
	pset2_m = engGetVariable(ep,"pset2_m");
	GoodFrames1_m = engGetVariable(ep,"GoodFrames1_m");
	GoodFrames2_m = engGetVariable(ep,"GoodFrames2_m");
	GoodDescriptor1_m = engGetVariable(ep,"GoodDescriptor1_m");
	GoodDescriptor2_m = engGetVariable(ep,"GoodDescriptor2_m");
	SolutionState_m = engGetVariable(ep,"SolutionState_m");

	if ((rotMat_m == NULL)||(transMat_m == NULL)||(SolutionState_m == NULL)) 
	{
		//LOGD<<"Get Array Failed for output of vodometry_c(...)";
	}
	else 
	{		
		rot_real = mxGetPr(rotMat_m);
		trans_real = mxGetPr(transMat_m); 
		psn_real = mxGetPr(psn_m);
		pset1_real = mxGetPr(pset1_m);
		pset2_real = mxGetPr(pset2_m);
		GoodFrames1_real = mxGetPr(GoodFrames1_m);
		GoodFrames2_real = mxGetPr(GoodFrames2_m);
		GoodDescriptor1_real = mxGetPr(GoodDescriptor1_m);
		GoodDescriptor2_real = mxGetPr(GoodDescriptor2_m);
		SolutionState_real = mxGetPr(SolutionState_m);

		//Reserve fixed length matrices
		cv::Mat rotMat=Mat::eye(3,3,CV_32FC1);
		cv::Mat transMat=Mat::zeros(3,1,CV_32FC1);

		if (rot_real)
		{
			int index=0;
			for (int j=0;j<3;j++)
				for (int i=0;i<3;i++)
				{
					rotMat.at<float>(i,j)=rot_real[index]; 
					index++;
				}
			//LOGD<<"rotMat:";
			//LOGMAT(rotMat);
		}

		if (trans_real)
		{
			int index=0;
			for (int j=0;j<3;j++)
			{
				transMat.at<float>(j,0)=trans_real[index]; 
				index++;
			}
			//LOGD<<"transMat:";
			//LOGMAT(transMat);
		}
		
		if (psn_real)
		{
			psn=(int)psn_real[0];
			//LOGD<<"state:"<<state;
		}

		if (SolutionState_real)
		{
			SolutionState=(int)SolutionState_real[0];
			state=SolutionState;
			//LOGD<<"SolutionState:"<<SolutionState;
		}

		//Reserve Matrices
		/*		
		rs_obj.pset1.create(3,psn,CV_32FC1);
		rs_obj.pset2.create(3,psn,CV_32FC1);
		cv::Mat GoodFrames1=Mat::zeros(2,psn,CV_32FC1);
		cv::Mat GoodFrames2=Mat::zeros(2,psn,CV_32FC1);
		cv::Mat GoodDescriptor1=Mat::zeros(128,psn,CV_32FC1);
		cv::Mat GoodDescriptor2=Mat::zeros(128,psn,CV_32FC1);
		*/
		if (SolutionState == 1)
		{
			/*			
			if (pset1_real)
			{
				int index=0;
				for (int j=0;j<psn;j++)
					for (int i=0;i<3;i++)
					{
						rs_obj.pset1.at<float>(i,j)=pset1_real[index]; //TBD:check order
						index++;
					}
				//LOGD<<"pset1:";
				//LOGMAT(pset1);
			}

			if (pset2_real)
			{
				int index=0;
				for (int j=0;j<psn;j++)
					for (int i=0;i<3;i++)
					{
						rs_obj.pset2.at<float>(i,j)=pset2_real[index]; //TBD:check order
						index++;
					}
				//LOGD<<"pset2:";
				//LOGMAT(pset2);
			}

			if (GoodFrames1_real)
			{
				int index=0;
				for (int j=0;j<psn;j++)
					for (int i=0;i<2;i++)
					{
						GoodFrames1.at<float>(i,j)=GoodFrames1_real[index]; //TBD:check order
						index++;
					}
				//LOGD<<"GoodFrames1:";
				//LOGMAT(GoodFrames1);
			}

			if (GoodFrames2_real)
			{
				int index=0;
				for (int j=0;j<psn;j++)
					for (int i=0;i<2;i++)
					{
						GoodFrames2.at<float>(i,j)=GoodFrames2_real[index]; //TBD:check order
						index++;
					}
				//LOGD<<"GoodFrames2:";
				//LOGMAT(GoodFrames2);
			}

			if (GoodDescriptor1_real)
			{
				int index=0;
				for (int j=0;j<psn;j++)
					for (int i=0;i<128;i++)
					{
						GoodDescriptor1.at<float>(i,j)=GoodDescriptor1_real[index]; //TBD:check order
						index++;
					}
				//LOGD<<"GoodDescriptor1:";
				//LOGMAT(GoodDescriptor1);
			}

			if (GoodDescriptor2_real)
			{
				int index=0;
				for (int j=0;j<psn;j++)
					for (int i=0;i<128;i++)
					{
						GoodDescriptor2.at<float>(i,j)=GoodDescriptor2_real[index]; //TBD:check order
						index++;
					}
				//LOGD<<"GoodDescriptor2:";
				//LOGMAT(GoodDescriptor2);
			}
		
			rs_obj.GoodFrames1.clear();
			for (int j=0;j<GoodFrames1.cols;j++)
			{
				cv::KeyPoint temp;
				temp.pt.x=GoodFrames1.at<float>(0,j)-1.0;
				temp.pt.y=GoodFrames1.at<float>(1,j)-1.0;
				rs_obj.GoodFrames1.push_back(temp);
			}

			rs_obj.GoodFrames2.clear();
			for (int j=0;j<GoodFrames2.cols;j++)
			{
				cv::KeyPoint temp;
				temp.pt.x=GoodFrames2.at<float>(0,j)-1.0;
				temp.pt.y=GoodFrames2.at<float>(1,j)-1.0;
				rs_obj.GoodFrames2.push_back(temp);
			}
			

			Mat gd1,gd2;
			cv::transpose(GoodDescriptor1,gd1);
			rs_obj.GoodDescp1.create(gd1.rows,gd1.cols,gd1.type());
			gd1.copyTo(rs_obj.GoodDescp1); //psnx128

			cv::transpose(GoodDescriptor2,gd2);
			rs_obj.GoodDescp2.create(gd2.rows,gd2.cols,gd2.type());
			gd2.copyTo(rs_obj.GoodDescp2); //psnx128

			for (int j=0;j<rs_obj.pset1.cols;j++)
			{
				cv::Point3f pt;
				pt.x=rs_obj.pset1.at<float>(0,j);
				pt.y=rs_obj.pset1.at<float>(1,j);
				pt.z=rs_obj.pset1.at<float>(2,j);
				rs_obj.op_pset1.push_back(pt);
			}

			for (int j=0;j<rs_obj.pset2.cols;j++)
			{
				cv::Point3f pt;
				pt.x=rs_obj.pset2.at<float>(0,j);
				pt.y=rs_obj.pset2.at<float>(1,j);
				pt.z=rs_obj.pset2.at<float>(2,j);
				rs_obj.op_pset2.push_back(pt);
			}*/
		}else
		{
			transMat=Mat::zeros(3,1,CV_32FC1);
			rotMat=Mat::eye(3,3,CV_32FC1);
		}

		rs_obj.transMat=transMat;
		rs_obj.rotMat=rotMat;
		rs_obj.q=r2q(rotMat);
		rs_obj.status=state;

		mxDestroyArray(rotMat_m);
		mxDestroyArray(transMat_m);
		mxDestroyArray(psn_m);
		mxDestroyArray(pset1_m);
		mxDestroyArray(pset2_m);
		mxDestroyArray(GoodFrames1_m);
		mxDestroyArray(GoodFrames2_m);
		mxDestroyArray(GoodDescriptor1_m);
		mxDestroyArray(GoodDescriptor2_m);
		mxDestroyArray(SolutionState_m);
	} 
	mxDestroyArray(x1_m);
	mxDestroyArray(x2_m);
	mxDestroyArray(y1_m);
	mxDestroyArray(y2_m);
	mxDestroyArray(z1_m);
	mxDestroyArray(z2_m);
	mxDestroyArray(img1_m);
	mxDestroyArray(img2_m);

	return rs_obj;
}
//******************************************************************************************************************
void imu_dataCB(const sensor_msgs::Imu::ConstPtr& imu_data)
{
	//ROS_INFO("I receive imu data %d", ++imu_count); 
	// display what I have received, 
  	{
		pthread_mutex_lock(&quat_mut_); 
			//cout<<tf::getYaw(imu_data->orientation);
			//cout<<"\n x:"<<imu_data->orientation.x<<" y:"<<imu_data->orientation.y<<" z:"<<imu_data->orientation.z<<"\n";
			quat_=imu_data->orientation;
			imu_stamp=imu_data->header.stamp;
			quat_is_ready = true;
		pthread_mutex_unlock(&quat_mut_);
	}
}
//*****************************************************************************************************************
geometry_msgs::Quaternion get_closest_imu(ros::Time stamp)
{
	geometry_msgs::Quaternion quat_local;
	quat_local.w=1.0;
	quat_local.x=0.0;
	quat_local.y=0.0;
	quat_local.z=0.0;

	for (int i=0; i<stacked_imu_stamps.size(); i++)
	{
		if (abs((stacked_imu_stamps.at(i) - stamp).toSec()) <= 1.0)
		{
			return stacked_vo_imu->at(i);
		}
	}
	return quat_local;
}
//*****************************************************************************************************************
void *vo_calc(void *ptr)
{
	ros::NodeHandle n;
	ros::Publisher vo_pub=n.advertise<nav_msgs::Odometry>("/vo", 60);
	tf::TransformBroadcaster odom_broadcaster;
	//ros::Publisher imu_pub=n.advertise<sensor_msgs::Imu>("/imu_vo", 60); //topic for sr_subscriber
	ros::Publisher imu_pub=n.advertise<sensor_msgs::Imu>("/imu_data", 60); //topic for robot_pose_ekf
	tf::TransformBroadcaster Iodom_broadcaster;
	
	ros::Rate loop_rate(25);
	//int step=5;
	geometry_msgs::Quaternion temp_q;
	temp_q.w=1.0;
	temp_q.x=0.0;
	temp_q.y=0.0;
	temp_q.z=0.0;
	//cv::Mat imu_quat=cv::Mat::zeros(4,1,CV_32FC1);
	cv::Mat imu_rot=cv::Mat::zeros(3,3,CV_32FC1);;
	char key;
	cv::namedWindow("intensity window", cv::WINDOW_AUTOSIZE);// Create a window for display.

	//************* Switch VO quat from IMU quat Declarations *******************
	tf::Quaternion tf_quat_imu;
	double yaw_cam, pitch_cam, roll_cam;
	double yaw_imu, pitch_imu, roll_imu;
	double lyaw_imu, lpitch_imu, lroll_imu;
	bool once=true;
	 
	double jump=2;

	while ((proc_obs_no-1 < obs_no) || ((proc_obs_no-1 == -1) && (obs_no == -1)) )
	{		
		//cout<<"\nproc_obs_no:"<<proc_obs_no<<" obs_no:"<<obs_no<<"\n";		
		ransac_stat rs_obj;		
		//if ((obs_no > 0) && (obs_no > proc_obs_no))
		if ((obs_no > 0) && (obs_no+1 > proc_obs_no+jump))
		{						
			proc_obs_no=proc_obs_no+jump;			
			//******************** Replace rot by IMU *************************
			//imu_quat=get_closest_imu(stacked_vo_stamps.at(proc_obs_no));
			if (proc_obs_no <= stacked_vo_imu->size()-1)
			{
				temp_q=stacked_vo_imu->at(proc_obs_no);
				/*imu_quat.at<float>(0,0)=temp_q.w;
				imu_quat.at<float>(1,0)=temp_q.x;
				imu_quat.at<float>(2,0)=temp_q.y;
				imu_quat.at<float>(3,0)=temp_q.z;*/
			}
			//imu_rot=q2r(imu_quat);
			//*****************************************************************	
			//******************** Publish IMU ********************************
			ros::Time current_time = ros::Time::now();		
		
			//first, we'll publish the transform over tf
	 		tf::StampedTransform transform_imu;

			//Transformation necessary to the robot_pose_ekf node
			transform_imu.stamp_=current_time;
			transform_imu.frame_id_="base_footprint";
			transform_imu.child_frame_id_ = "/imu_frame";        
			tf::Vector3 transl_imu(0,0,0);
			transl_imu[0]=0; 
			transl_imu[1]=0; 
			transl_imu[2]=0;
			transform_imu.setOrigin(transl_imu);

			tf::Quaternion odom_quat_imu = tf::createQuaternionFromRPY(0,0,0);
			transform_imu.setRotation(odom_quat_imu);
			//Publish tf
			Iodom_broadcaster.sendTransform(transform_imu);
			//***********************END TF TRANSFORM FOR IMU*******************
	  		//next, we'll publish the IMU message over ROS
			//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(yaw,-pitch,-roll);

	    		sensor_msgs::Imu Iodom;
	    		Iodom.header.stamp = current_time;
			Iodom.header.frame_id = "/imu_frame";	
			//************* Switch VO quat from IMU quat *******************
			quaternionMsgToTF(temp_q, tf_quat_imu);
			tf::Matrix3x3(tf_quat_imu).getEulerYPR(yaw_imu,pitch_imu,roll_imu); //take care of seq of execution
			tf::Matrix3x3 rotMat_imu;			
			if (once == true)
			{
				lpitch_imu=pitch_imu;
				lyaw_imu=yaw_imu;
				lroll_imu=roll_imu;
				once = false;
			}	
			//************** Adjust the 180 to -180 adjustment jump in IMU output *************
			if (lpitch_imu-pitch_imu > 5)
			{
				pitch_imu=pitch_imu+6.28319;
			}else if (pitch_imu-lpitch_imu > 5)
			{
				pitch_imu=pitch_imu-6.28319;
			}

			if (lroll_imu-roll_imu > 5)
			{
				roll_imu=roll_imu+6.28319;
			}else if (roll_imu-lroll_imu > 5)
			{
				roll_imu=roll_imu-6.28319;
			}

			if (lyaw_imu-yaw_imu > 5)
			{
				yaw_imu=yaw_imu+6.28319;
			}else if (yaw_imu-lyaw_imu > 5)
			{
				yaw_imu=yaw_imu-6.28319;
			}
			//***** End Adjust the 180 to -180 adjustment jump in IMU output ******
			
			rotMat_imu.setEulerYPR(-roll_imu+lroll_imu,yaw_imu-lyaw_imu,-pitch_imu+lpitch_imu);//ypr//rpy//pry//pyr//yrp//ryp//r-yp//-ryp(better)//-ry-p(more better)
			geometry_msgs::Quaternion gm_q=tf::createQuaternionMsgFromRollPitchYaw(-roll_imu+lroll_imu,yaw_imu-lyaw_imu,-pitch_imu+lpitch_imu);
			//********** Convert TFMatrix to OpenCV Mat via EigMat ********
			Eigen::Matrix3d rotEig;	
			tf::matrixTFToEigen(rotMat_imu, rotEig);
			for (int i=0;i<3;i++)
				for (int j=0;j<3;j++)
				{
					imu_rot.at<float>(i,j)=rotEig(i,j);
				}
			//**************************************************************
			Iodom.orientation=gm_q;
			Iodom.orientation_covariance[0]= 0.000001;
			Iodom.orientation_covariance[4]= 0.000001; 
			Iodom.orientation_covariance[8]= 0.000001;
			Iodom.angular_velocity_covariance[0]= 0.000304617;
			Iodom.angular_velocity_covariance[4]= 0.000304617;
			Iodom.angular_velocity_covariance[8]= 0.000304617;
			Iodom.linear_acceleration_covariance[0]= 0.0000004;
			Iodom.linear_acceleration_covariance[4]= 0.0000004;
			Iodom.linear_acceleration_covariance[8]= 0.0000004;

			//publish the message
			imu_pub.publish(Iodom);
			//******************* Calculate VO *****************************								
			ROS_INFO("ekf_subscriber.cpp: Before vodom for frame no. %d",proc_obs_no);
			while(vo_is_ready == false)
			{}
			rs_obj=vodometery_matlab(xs.at(proc_obs_no-jump), ys.at(proc_obs_no-jump), zs.at(proc_obs_no-jump), xs.at(proc_obs_no), ys.at(proc_obs_no), zs.at(proc_obs_no), I_imgs.at(proc_obs_no-jump), I_imgs.at(proc_obs_no), state, ep, proc_obs_no, imu_rot); 
			//rs_obj=vodometery_matlab(xs.at(proc_obs_no-1), ys.at(proc_obs_no-1), zs.at(proc_obs_no-1), xs.at(proc_obs_no), ys.at(proc_obs_no), zs.at(proc_obs_no), I_imgs.at(proc_obs_no-1), I_imgs.at(proc_obs_no), state, ep, proc_obs_no); 
			ROS_INFO("ekf_subscriber.cpp: After vodom for frame no. %d",proc_obs_no);
			//******************** Publish VO *******************************	
			//first, we'll publish the transform over tf
	 		tf::StampedTransform transform_vo;

			//current_time = ros::Time::now(); //advance timestamp for vo by a little bit
			//Transformation necessary to the robot_pose_ekf node
			transform_vo.stamp_=current_time;
			transform_vo.frame_id_="base_footprint";
			transform_vo.child_frame_id_ = "/odom";        
			tf::Vector3 transl_vo(0,0,0);
			transl_vo[0]=0; 
			transl_vo[1]=0; 
			transl_vo[2]=0;
			transform_vo.setOrigin(transl_vo);

			tf::Quaternion odom_quat_vo = tf::createQuaternionFromRPY(0,0,0);
			transform_vo.setRotation(odom_quat_vo);
			//Publish tf
			odom_broadcaster.sendTransform(transform_vo);
			//***********************END TF TRANSFORM FOR VO***********************
	   		//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
	    		odom.header.stamp = current_time;
			odom.header.frame_id = "/odom";		

			//set the position
			odom.pose.pose.position.x = rs_obj.transMat.at<float>(0,0);
			odom.pose.pose.position.y = rs_obj.transMat.at<float>(1,0);
			odom.pose.pose.position.z = rs_obj.transMat.at<float>(2,0);
			geometry_msgs::Quaternion odom_quat;
			
			tf::Quaternion tfq=tf::Quaternion(rs_obj.q.at<float>(1,0), rs_obj.q.at<float>(2,0), rs_obj.q.at<float>(3,0), rs_obj.q.at<float>(0,0));		
			tf::quaternionTFToMsg (tfq, odom_quat);
			odom.pose.pose.orientation = odom_quat;
			tf::Matrix3x3(tfq).getEulerYPR(yaw_cam,pitch_cam,roll_cam); //take care of seq of execution
			//compute odometry in a typical way given the velocities of the robot
			//double dt = (current_time2 - last_time2).toSec();
			//set the velocity
			//odom.twist.twist.linear.x = 0.1; //TBD: calculated via dist/time or s=vt formula
			//odom.twist.twist.linear.y = 0.1;
			//odom.twist.twist.angular.z = 1.0;

			// covariance
			for (unsigned int i=0; i<6; i++)
				for (unsigned int j=0; j<6; j++)
					if (i == j)
						odom.pose.covariance[6*i+j] = pow(0.00017,2);
	
			//publish the message
			vo_pub.publish(odom);
			//***************************************************************************************
			//pthread_mutex_lock(&vo_mut_);
			//	vo_is_ready = false;			
			//pthread_mutex_unlock(&vo_mut_);

			//********************** Update logged poses ***********************
			//ROS_INFO("Marker 1");	
			update_vo(stacked_vo_cam,imu_rot,rs_obj.transMat);
			//ROS_INFO("Marker 2");
			//cout<<"\nimu roll:"<<-pitch_imu+lpitch_imu<<" imu_pitch:"<<yaw_imu-lyaw_imu<<" imu_yaw:"<<-roll_imu+lroll_imu<<"\n";
			//ROS_INFO("Marker 3");
			update_rpy(stacked_rpy_imu,-pitch_imu+lpitch_imu,yaw_imu-lyaw_imu,-roll_imu+lroll_imu);
			//ROS_INFO("Marker 4");
			update_rpy(stacked_rpy_cam,roll_cam,pitch_cam,yaw_cam);
			//ROS_INFO("Marker 5");
			lpitch_imu=pitch_imu;
			lyaw_imu=yaw_imu;
			lroll_imu=roll_imu;
			//************* END Switch VO quat from IMU quat *******************
			//update_vo(stacked_vo_cam,rs_obj.rotMat,rs_obj.transMat);			
			cv::imshow( "intensity window", I_imgs.at(proc_obs_no));   // Show our image inside it.
		}else
		{	
			//pthread_mutex_lock(&vo_mut_);
			//	vo_is_ready = false;			
			//pthread_mutex_unlock(&vo_mut_);
		}
		//ros::spinOnce();
		//usleep(10000); // sleep 10 ms
		loop_rate.sleep();
		key=cvWaitKey(20);

		if (key == 'q') 
		{
			break;
		}
	}
	cvDestroyWindow("intensity window");
	save_stacked_rpy(*(stacked_rpy_imu),"/home/emaad22/Desktop/initial result/rosbag2/IMU_quat+VO_trans/rpyimu.csv");
	save_stacked_rpy(*(stacked_rpy_cam),"/home/emaad22/Desktop/initial result/rosbag2/IMU_quat+VO_trans/rpycam.csv");
	save_stacked_vo(*(stacked_vo_cam),"/home/emaad22/Desktop/initial result/rosbag2/IMU_quat+VO_trans/vopath.csv");
	//save_stacked_vo(*(stacked_vo_cam),"/home/emaad22/Desktop/vopath.csv"); //debug: for path-visualization in matlab only
	pthread_mutex_lock(&quit_mut_);
		quit = true;
	pthread_mutex_unlock(&quit_mut_);
}
//*****************************************************************************************************************
void sr_arrayCB_pcl(const std_msgs::UInt8MultiArray::ConstPtr& sr_array)
{
	//ROS_INFO("I receive sr4000 data %d", ++g_count);	
	static bool once = false; 
	if(!once)
	{
		g_buf_.resize(sr_array->data.size()); 
		once = true; 
	}
	{
		pthread_mutex_lock(&g_mut_); 
			memcpy(g_buf_.data(), sr_array->data.data(), g_buf_.size());
			g_cloud_is_ready = true;
		pthread_mutex_unlock(&g_mut_);
		//cam_stamp=sr_array->header.stamp;
	}
}
//******************************************************************************************************************
// get a new intensity image and depth image
bool getUpdateImgnCld(cv::Mat& i_img, cv::Mat& d_img, cloudPtr& pc)
{
	// get raw data from sr_publisher 
	if(!g_cloud_is_ready) 
	{
		//ROS_INFO("sr_subscriber.cpp: Image is not ready, wait!");
		return false; 
	}

	static bool once = false;
	static vector<unsigned char> local_buf;
	
	// camera model without distortion parameters
	static CamModel cam_model(223.9758, 226.7442, 89.361, 75.8112);

	if(!once)
	{
		local_buf.resize(g_buf_.size());
		once = true; 
	} 
	{
		pthread_mutex_lock(&g_mut_); 
			// pc = g_point_cloud->makeShared(); 
			memcpy(local_buf.data(), g_buf_.data(), local_buf.size());
			g_cloud_is_ready = false;
		pthread_mutex_unlock(&g_mut_);
	}

	{
		// construct a point cloud 
		unsigned char* pDis = local_buf.data(); 

		// compute the point cloud
		int rows = 144; 
		int cols = 176;
		unsigned int total = rows*cols; 
		unsigned int sr_n = total;

		const static unsigned int distance_size = sr_n*(sizeof(unsigned short) + sizeof(unsigned char));
		const static unsigned int coordinate_size = sr_n*(sizeof(float)*3 + sizeof(unsigned char));
		if(local_buf.size() == distance_size)
		{
			unsigned short* pD = (unsigned short*)(pDis); 
			unsigned char* pI = (pDis + total*sizeof(unsigned short));

			double ox, oy, oz;
			if(pc->points.size() != total)
			{
				pc->points.resize(total); 
			}
			for(int i=0; i<rows; i++)
			{
				for(int j=0; j<cols; j++)
				{
					int k = i*cols + j; 
					float z = (*pD)*0.001; 
					cam_model.convertUVZ2XYZ(j+0.5, i+0.5, z, ox, oy, oz); 
					point_type& pt = pc->points[k];
					pt.x = -ox; pt.y = -oy; pt.z = oz - 0.01;
					++pD;
					++pI;
				}
			}
		}else
		{
			unsigned char* pI = pDis;
			float * px = (float*)(pDis + sr_n*sizeof(unsigned char)); 
			float * py = (float*)(pDis + sr_n*(sizeof(unsigned char) + sizeof(float))); 
			float * pz = (float*)(pDis + sr_n*(sizeof(unsigned char) + sizeof(float)*2));
			for(int i=0; i<total; i++)
			{
				point_type& pt = pc->points[i]; 
				pt.x = *px; pt.y = *py; pt.z = *pz; 
				++px; ++py; ++pz;
			}
		}
	}
	// get distance image first 
	unsigned char* pDis = local_buf.data(); 

	// compute the point cloud
	int rows = 144; 
	int cols = 176;
	unsigned int total = rows*cols; 
	unsigned int sr_n  = total;
	unsigned short* pD ; 
	unsigned char* pI ; 

	cv::Mat intensity_img;   
	cv::Mat distance_img; 
	const static unsigned int distance_size = sr_n*(sizeof(unsigned short) + sizeof(unsigned char));
	const static unsigned int coordinate_size = sr_n*(sizeof(float)*3 + sizeof(unsigned char));
	if(local_buf.size() == distance_size)
	{
		pD = (unsigned short*)(pDis); 
		pI = (pDis + total*sizeof(unsigned short));
		intensity_img = cv::Mat(rows, cols, CV_8UC1, pI); 
		distance_img = cv::Mat(rows, cols, CV_16UC1, pD); 
	}else
	{
		pI = pDis; 
		intensity_img = cv::Mat(rows, cols, CV_8UC1, pI);
	}

	// copy out 
	i_img = intensity_img.clone(); 
	return true;
}
//******************************************************************************************************************
// get a new intensity image and depth image
bool getUpdateImage(cv::Mat& i_img, cv::Mat& d_img)
{
	// get raw data from sr_publisher 
	if(!g_cloud_is_ready) 
	{
		//ROS_INFO("sr_subscriber.cpp: Image is not ready, wait!");
		return false; 
	}

	static bool once = false;
	static vector<unsigned char> local_buf;

	if(!once)
	{
		local_buf.resize(g_buf_.size());
		once = true; 
	} 
	{
		pthread_mutex_lock(&g_mut_); 
			// pc = g_point_cloud->makeShared(); 
			memcpy(local_buf.data(), g_buf_.data(), local_buf.size());
			g_cloud_is_ready = false;
		pthread_mutex_unlock(&g_mut_);
	}

	// get distance image first 
	unsigned char* pDis = local_buf.data(); 

	// compute the point cloud
	int rows = 144; 
	int cols = 176;
	unsigned int total = rows*cols; 
	unsigned int sr_n  = total;
	unsigned short* pD ; 
	unsigned char* pI ; 

	cv::Mat intensity_img;   
	cv::Mat distance_img; 
	const static unsigned int distance_size = sr_n*(sizeof(unsigned short) + sizeof(unsigned char));
	const static unsigned int coordinate_size = sr_n*(sizeof(float)*3 + sizeof(unsigned char));
	if(local_buf.size() == distance_size)
	{
		pD = (unsigned short*)(pDis); 
		pI = (pDis + total*sizeof(unsigned short));
		intensity_img = cv::Mat(rows, cols, CV_8UC1, pI); 
		distance_img = cv::Mat(rows, cols, CV_16UC1, pD); 
	}else
	{
		pI = pDis; 
		intensity_img = cv::Mat(rows, cols, CV_8UC1, pI);
	}

	// copy out 
	i_img = intensity_img.clone(); 
	return true;
}
//************************************************************************************************************************
//get imu data
bool getUpdateIMU(geometry_msgs::Quaternion& local_quat)
{
	try
	{
		if (!quat_is_ready)
		{
			//ROS_INFO("ekf_subscriber.cpp: quat is not ready, wait!");
			return false; 
		}

		{
			pthread_mutex_lock(&quat_mut_); 
				//tf::quaternionMsgToTF(quat_, local_quat);
				local_quat=quat_;
				quat_is_ready = false;
			pthread_mutex_unlock(&quat_mut_);	
		}
		return true;
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception trying to display subscribed info: %s", ex.what());
	}
}
//************************************************************************************************************************
// get a new point cloud
bool getUpdateCloud(cloudPtr& pc)
{
	try
	{
		if(!g_cloud_is_ready) //implement a timestamp distance check here
		{
			//ROS_INFO("sr_subscriber.cpp: cloud is not ready, wait!");
			return false; 
		}
		static bool once = false;
		static vector<unsigned char> local_buf;

		// camera model without distortion parameters
		static CamModel cam_model(223.9758, 226.7442, 89.361, 75.8112);
		if(!once)
		{
			local_buf.resize(g_buf_.size());
			once = true; 
		}
		{
			pthread_mutex_lock(&g_mut_); 
				memcpy(local_buf.data(), g_buf_.data(), local_buf.size());
				g_cloud_is_ready = false;
			pthread_mutex_unlock(&g_mut_);
		}

		{
			// construct a point cloud 
			unsigned char* pDis = local_buf.data(); 

			// compute the point cloud
			int rows = 144; 
			int cols = 176;
			unsigned int total = rows*cols; 
			unsigned int sr_n = total;

			const static unsigned int distance_size = sr_n*(sizeof(unsigned short) + sizeof(unsigned char));
			const static unsigned int coordinate_size = sr_n*(sizeof(float)*3 + sizeof(unsigned char));
			if(local_buf.size() == distance_size)
			{
				unsigned short* pD = (unsigned short*)(pDis); 
				unsigned char* pI = (pDis + total*sizeof(unsigned short));

				double ox, oy, oz;
				if(pc->points.size() != total)
				{
					pc->points.resize(total); 
				}
				for(int i=0; i<rows; i++)
				{
					for(int j=0; j<cols; j++)
					{
						int k = i*cols + j; 
						float z = (*pD)*0.001; 
						cam_model.convertUVZ2XYZ(j+0.5, i+0.5, z, ox, oy, oz); 
						point_type& pt = pc->points[k]; 
						pt.x = -ox; pt.y = -oy; pt.z = oz - 0.01;
						++pD;
						++pI;
					}
				}
			}else
			{
				unsigned char* pI = pDis;
				float * px = (float*)(pDis + sr_n*sizeof(unsigned char)); 
				float * py = (float*)(pDis + sr_n*(sizeof(unsigned char) + sizeof(float))); 
				float * pz = (float*)(pDis + sr_n*(sizeof(unsigned char) + sizeof(float)*2));
				for(int i=0; i<total; i++)
				{
					point_type& pt = pc->points[i]; 
					pt.x = *px; pt.y = *py; pt.z = *pz; 
					++px; ++py; ++pz;
				}
			}
		}
		return true;
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception trying to display subscribed info: %s", ex.what());
	}
}
//*******************************************************************************************************************
void start_loop(ros::NodeHandle& n)
{
	stacked_vo_cam= new std::vector<cv::Mat>();
	stacked_rpy_cam= new std::vector<tf::Vector3>();
	stacked_rpy_imu= new std::vector<tf::Vector3>();	
	stacked_vo_imu= new std::vector<geometry_msgs::Quaternion>();	
	// subscribe /sw_array 
	ros::Subscriber sr_array_sub = n.subscribe<std_msgs::UInt8MultiArray>("/sr_array", 500, sr_arrayCB_pcl);
	// subscribe /imu_data 
	ros::Subscriber imu_data_sub = n.subscribe<sensor_msgs::Imu>("/imu_vo", 500, imu_dataCB); //using remapped rosbag. use /imu_data for unremapped rosbad (28 Sep 2016)

	// spawn another thread
	pthread_t thread_vo;
	
	int rc1;
	/*Create independent threads each of which will execute functionC */
	if( (rc1=pthread_create( &thread_vo, NULL, &vo_calc, NULL)) )
   	{
		printf("Thread creation failed: %d\n", rc1);
	}

	geometry_msgs::Quaternion local_quat;
	
	cv::Mat I_img1; 
	cv::Mat D_img1;
	cv::Mat x1=cv::Mat(144,176,CV_32FC1);
	cv::Mat y1=cv::Mat(144,176,CV_32FC1);
	cv::Mat z1=cv::Mat(144,176,CV_32FC1);

	// PCL point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<point_type>(144, 176));	
	
	/*
		*** Start the MATLAB engine 
	*/
	if (!(ep = engOpen(NULL))) {
		//LOGD<<"Can't start MATLAB engine in featurehandler.cpp";
		exit(-1);
	}

	while((n.ok()) && (quit == false))
	{
		/*if(getUpdateCloud(cloud))
		{
			//ROS_INFO("sr_subscriber.cpp: great I get a new cloud, show it!");
			//cin.get();
		}*/

		if(getUpdateIMU(local_quat))
		{
			//ROS_INFO("sr_subscriber.cpp: great I get a new IMU data, show it!");
			//cin.get();
			stacked_vo_imu->push_back(local_quat);
			stacked_imu_stamps.push_back(imu_stamp);
		}
		
		if (getUpdateImgnCld(I_img1, D_img1, cloud1)) 
		//if(getUpdateImage(I_img1, D_img1))
		{
			//cout<<"\n***"<<cloud1->points.size ()<<"***\n";
			//cout<<"\n***"<<cloud1->height<<"x"<<cloud1->width<<"***\n";
			//cin.get();
			for (int i=0; i<cloud1->width; ++i)
				for (int j=0; j<cloud1->height; ++j)
				{
					x1.at<float>(i,j)=cloud1->points[(i*cloud1->height)+j].x;
					y1.at<float>(i,j)=cloud1->points[(i*cloud1->height)+j].y;
					z1.at<float>(i,j)=cloud1->points[(i*cloud1->height)+j].z;
					//img1.at<float>(i,j)=I_img1.at<float>(i,j); //cv::Mat(rows144, cols176, CV_8UC1, pI); 
				}
		
			pthread_mutex_lock(&vo_mut_);
				vo_is_ready=false;			
			pthread_mutex_unlock(&vo_mut_);
				xs.push_back(x1);
				ys.push_back(y1);
				zs.push_back(z1);
				I_imgs.push_back(I_img1);
				//stacked_vo_stamps.push_back(cam_stamp);
				obs_no++;
			pthread_mutex_lock(&vo_mut_);
				vo_is_ready=true;			
			pthread_mutex_unlock(&vo_mut_); 	
		}

		ros::spinOnce();
		usleep(10000); // sleep 10 ms
	}
	pthread_join(thread_vo, NULL);
}
//*******************************************************************************************************************
int main(int argc, char** argv)
{
	ros::init(argc, argv, "ekf_subscriber");

	ros::NodeHandle n;
	pthread_mutex_init(&g_mut_, NULL);
	pthread_mutex_init(&quat_mut_, NULL);
	pthread_mutex_init(&vo_mut_, NULL);
	start_loop(n);
	return 0;
}
