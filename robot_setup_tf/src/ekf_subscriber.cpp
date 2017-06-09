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

//#include "sr4000handler.cpp"
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

#include "engine.h" //for matlab
#include "matrix.h" //for matlab

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
//#include "pcl/visualization/pcl_visualizer.h"

#define OBS_JUMP 1
#define IMU_CAM_OBS_TIMEGAP 0.1
#define IMUTRANS_ANG_THRESH 0.03//0.07
#define USE_IMU_ROT 1
#define USE_IMU_TRANS 0
#define DATASET_PATH "/home/emaad22/Desktop/initial result/rosbag7/"
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
bool srtim_is_ready = false;
bool quat_is_ready = false;
bool vo_is_ready = false;
bool quit = false;

pthread_mutex_t vo_mut_;
pthread_mutex_t g_mut_;
pthread_mutex_t stacks_mut_;
pthread_mutex_t quat_mut_;
pthread_mutex_t quit_mut_;
pthread_mutex_t srtim_mut_;
vector<unsigned char> g_buf_;
geometry_msgs::Quaternion quat_;
geometry_msgs::Vector3 acc_;

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
std::vector<ros::Time>  *stacked_cam_stamps;
ros::Time cam_stamp;

std::vector<geometry_msgs::Quaternion> *stacked_q_imu; //for saving 4x1 quat trace
std::vector<geometry_msgs::Vector3> *stacked_acc_imu; //for saving 3x1 gravity compensated acc trace
std::vector<ros::Time>  *stacked_imu_stamps;
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
	
	tf::Quaternion q_tf(q.at<float>(1,0), q.at<float>(2,0), q.at<float>(3,0), q.at<float>(0,0));
	tf::Matrix3x3 m(q_tf);

	Eigen::Matrix3d rotEig;	
	tf::matrixTFToEigen(m, rotEig);
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
		{
			R.at<float>(i,j)=rotEig(i,j);
		}

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
		Mat inv_elem=elem.inv();

		for (int j=0;j<inv_elem.rows;j++)
		{
			myfile<<inv_elem.at<float>(j,3);
			if (j<inv_elem.rows-1)
				myfile<<",";
		}
		myfile<<"\n";
	}
	
	myfile.close();
}
//*******************************************************************************************************************
void save_stacked_vo_kitti(std::vector<cv::Mat> stacked_vo, std::string path)
{
	ofstream myfile;
	myfile.open (path);
	for(int i=0;i<stacked_vo.size();i++)
	{
		Mat elem=stacked_vo.at(i);
		Mat inv_elem=elem.inv();
		inv_elem.at<float>(2,3)=-inv_elem.at<float>(2,3);
		for (int j=0;j<inv_elem.rows-1;j++)
			for (int k=0;k<inv_elem.cols;k++)
			{
				if ((j == inv_elem.rows-2) && (k == inv_elem.cols-1))
					myfile<<inv_elem.at<float>(j,k)<<"\n";
				else
					myfile<<inv_elem.at<float>(j,k)<<" ";	
			}
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
	int last_idx;
	Mat last_pose;
	if (vo->size()==0)
	{
		last_pose=Mat::eye(4,4,CV_32FC1);
	}else
	{
		last_idx=vo->size()-1;
		last_pose=vo->at(last_idx); //TBD: are we really picking from last pose or we pick from first ever pose
	} 

	//rot=rot.inv();///////////////

	Mat transf_mat=Mat::eye(4,4,CV_32FC1);
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			transf_mat.at<float>(i,j)=rot.at<float>(i,j); //3x3 patch update
	
	transf_mat.at<float>(0,3)=trans.at<float>(0,0); //3x1 patch update
	transf_mat.at<float>(1,3)=trans.at<float>(1,0); //3x1 patch update
	transf_mat.at<float>(2,3)=trans.at<float>(2,0); //3x1 patch update

	//Mat inv_transf=transf_mat.inv();
	//Mat latest_pose=inv_transf*last_pose;
		
	Mat latest_pose=transf_mat*last_pose;
	
	vo->push_back(latest_pose);
	/////////////////////////////////////////////////////////////////
}
//**************************************************************************************************************************
ransac_stat vodometery_matlab(cv::Mat x1, cv::Mat y1, cv::Mat z1, cv::Mat x2, cv::Mat y2, cv::Mat z2, cv::Mat img1, cv::Mat img2, int &state, Engine *ep, int loc_obs_no, cv::Mat imu_rot)
{
	ransac_stat rs_obj; //create empty rs_obj
	int psn=0;
	int nMatches=0;
	int SolutionState=-1;

	//Define outputs
	mxArray *rotMat_m=NULL;
	mxArray *transMat_m=NULL;
	mxArray *SolutionState_m=NULL;
	mxArray *psn_m=NULL;
	mxArray *nMatches_m=NULL;
	mxArray *match_m=NULL;
	mxArray *GoodFrames1_m=NULL;
	mxArray *GoodFrames2_m=NULL;

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
	
	double *rot_real, *trans_real, *psn_real, *nMatches_real;
	double *GoodFrames1_real, *GoodFrames2_real, *match_real;
	double *SolutionState_real;

        // engEvalString(ep,"cd('/home/emaad22/Desktop/forth_height_EKF/code_from_dr_ye')");
        engEvalString(ep, "cd('/home/davidz/work/ros_hydro_ws/src/robot_setup_tf/matlab_code')");
	engPutVariable(ep, "x1_m", x1_m); 
	engPutVariable(ep, "x2_m", x2_m); 
	engPutVariable(ep, "y1_m", y1_m); 
	engPutVariable(ep, "y2_m", y2_m); 
	engPutVariable(ep, "z1_m", z1_m); 
	engPutVariable(ep, "z2_m", z2_m);
	engPutVariable(ep, "img1_m", img1_m); 
	engPutVariable(ep, "img2_m", img2_m); 
	engPutVariable(ep, "imurot_m", imurot_m); 

	if (USE_IMU_ROT == 1)	
		engEvalString(ep,"[rotMat_m, transMat_m, GoodFrames1_m, GoodFrames2_m, psn_m, nMatches_m, match_m, SolutionState_m] = vodometry_imumod(x1_m,x2_m,y1_m,y2_m,z1_m,z2_m,img1_m,img2_m,imurot_m)");
	else
		engEvalString(ep,"[rotMat_m, transMat_m, GoodFrames1_m, GoodFrames2_m, psn_m, nMatches_m, match_m, SolutionState_m] = vodometry_cammod(x1_m,x2_m,y1_m,y2_m,z1_m,z2_m,img1_m,img2_m)");

	rotMat_m = engGetVariable(ep,"rotMat_m");
	transMat_m = engGetVariable(ep,"transMat_m");
	psn_m = engGetVariable(ep,"psn_m"); 
	nMatches_m = engGetVariable(ep,"nMatches_m");
	match_m = engGetVariable(ep,"match_m"); 
	GoodFrames1_m = engGetVariable(ep,"GoodFrames1_m");
	GoodFrames2_m = engGetVariable(ep,"GoodFrames2_m");
	SolutionState_m = engGetVariable(ep,"SolutionState_m");

	if ((rotMat_m == NULL)||(transMat_m == NULL)||(SolutionState_m == NULL)) 
	{
		//LOGD<<"Get Array Failed for output of vodometry_c(...)";
	}
	else 
	{		
		rot_real = mxGetPr(rotMat_m);
		trans_real = mxGetPr(transMat_m); 
		SolutionState_real = mxGetPr(SolutionState_m);
		psn_real = mxGetPr(psn_m);
		nMatches_real = mxGetPr(nMatches_m);
		match_real = mxGetPr(match_m);
		GoodFrames1_real = mxGetPr(GoodFrames1_m);
		GoodFrames2_real = mxGetPr(GoodFrames2_m);

		//Reserve fixed length matrices
		cv::Mat rotMat=Mat::eye(3,3,CV_32FC1);
		cv::Mat transMat=Mat::zeros(3,1,CV_32FC1);

		if (nMatches_real)
		{
			nMatches=(int)nMatches_real[0];
			//LOGD<<"state:"<<state;
		}


		if (psn_real)
		{
			psn=(int)psn_real[0];
			//LOGD<<"state:"<<state;
		}

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
		
		if (SolutionState_real)
		{
			SolutionState=(int)SolutionState_real[0];
			state=SolutionState;
			//LOGD<<"SolutionState:"<<SolutionState;
		}

		if (SolutionState == 1)
		{
			cv::Mat GoodFrames1=Mat::zeros(2,psn,CV_32FC1);
			cv::Mat GoodFrames2=Mat::zeros(2,psn,CV_32FC1);
			cv::Mat match=Mat::zeros(2,nMatches,CV_32FC1);

			if (match_real)
			{
				int index=0;
				for (int j=0;j<nMatches;j++)
					for (int i=0;i<2;i++)
					{
						match.at<float>(i,j)=match_real[index]; //TBD:check order
						index++;
					}
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
			}

			rs_obj.match.clear();
			for (int j=0;j<match.cols;j++)
			{
				cv::KeyPoint temp;
				temp.pt.x=match.at<float>(0,j)-1.0;
				temp.pt.y=match.at<float>(1,j)-1.0;
				rs_obj.match.push_back(temp);
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
		mxDestroyArray(SolutionState_m);
		mxDestroyArray(GoodFrames1_m);
		mxDestroyArray(GoodFrames2_m);
		mxDestroyArray(match_m);
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
			acc_=imu_data->linear_acceleration;
			//acc_.x=imu_data->linear_acceleration_covariance[0];
			//acc_.y=imu_data->linear_acceleration_covariance[1];
			//acc_.x=imu_data->linear_acceleration_covariance[2];
			imu_stamp=imu_data->header.stamp;
			quat_is_ready = true;
		pthread_mutex_unlock(&quat_mut_);
	}

	//reference from sr_publisher from robot
	//Iodom.orientation_covariance[0]= vel_uncert;
	//Iodom.orientation_covariance[1]= att_uncert;
	//Iodom.linear_acceleration_covariance[0]= body_estvx;	
	//Iodom.linear_acceleration_covariance[1]= body_estvy;
	//Iodom.linear_acceleration_covariance[2]= body_estvz;
}
//*****************************************************************************************************************
int get_closest_imu(ros::Time stamp, int srch_st_point)
{
	for (int i=srch_st_point; i<stacked_imu_stamps->size(); i++)
	{
		//cout<<"\n stacked_imu_stamps->at(i).toSec():"<<stacked_imu_stamps->at(i).toSec()<<"\n";				
		//cout<<"\n stamp.toSec():"<<stamp.toSec()<<"\n";
		//cout<<"\n abs((stacked_imu_stamps->at(i) - stamp).toSec():"<<abs((stacked_imu_stamps->at(i) - stamp).toSec())<<"\n";		
		if (abs((stacked_imu_stamps->at(i) - stamp).toSec()) <= IMU_CAM_OBS_TIMEGAP)
		//if (stamp > stacked_imu_stamps->at(i))
		{
			return i;
		}
	}
	return srch_st_point;
}
//*****************************************************************************************************************
void rotate_vector_by_quaternion(const geometry_msgs::Vector3& v, const geometry_msgs::Quaternion& q, geometry_msgs::Vector3& vprime) //optimal function
{
	// Extract the vector part of the quaternion
	tf::Vector3 u_tf(q.x,q.y,q.z);
	tf::Vector3 v_tf(v.x,v.y,v.z);

    	// Extract the scalar part of the quaternion
    	double s = q.w;
	
	// Do the math
	double uvdot= u_tf.dot(v_tf);
	tf::Vector3 uvcross= u_tf.cross(v_tf);
	double uudot= u_tf.dot(u_tf);
	u_tf.setX(u_tf.getX()*2.0*uvdot);
	u_tf.setY(u_tf.getY()*2.0*uvdot);
	u_tf.setZ(u_tf.getZ()*2.0*uvdot);

	v_tf.setX((s*s-uudot)*v_tf.getX());
	v_tf.setY((s*s-uudot)*v_tf.getY());
	v_tf.setZ((s*s-uudot)*v_tf.getZ());
	
	uvcross.setX(2.0*s*uvcross.getX());
	uvcross.setY(2.0*s*uvcross.getY());
	uvcross.setZ(2.0*s*uvcross.getZ());


    	tf::Vector3 vp_tf = u_tf + v_tf + uvcross ;
	vprime.x= vp_tf.getX();
	vprime.y= vp_tf.getY();
	vprime.z= vp_tf.getZ();
}
//*****************************************************************************************************************
geometry_msgs::Vector3 get_gcomp_acc(geometry_msgs::Vector3 acc_in, geometry_msgs::Quaternion q_in)
{
	geometry_msgs::Vector3 acc_out;
	//Gravity compensate

	rotate_vector_by_quaternion(acc_in, q_in, acc_out);
	//Convert acceleration from G to m/s/s
	acc_out.x = acc_out.x* 9.8;
	acc_out.y = acc_out.y* 9.8;
	acc_out.z = acc_out.z* 9.8;

	return acc_out; //m/s/s
}
//*****************************************************************************************************************
void *vo_calc(void *ptr)
{
	ros::NodeHandle n;
	////ros::Publisher vo_pub=n.advertise<nav_msgs::Odometry>("/vo", 60);
	////tf::TransformBroadcaster odom_broadcaster;
	////ros::Publisher imu_pub=n.advertise<sensor_msgs::Imu>("/imu_data", 60); //topic for robot_pose_ekf
	////tf::TransformBroadcaster Iodom_broadcaster;
	
	double dbg_rpy[3];
	dbg_rpy[0]=0;
	dbg_rpy[1]=0;
	dbg_rpy[2]=0;
		
	ros::Rate loop_rate(25);

	geometry_msgs::Vector3 temp_acc;
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
	 
	ros::Time current_time, last_time=ros::Time::now();
	
	//***********************TF TRANSFORM FOR IMU*******************
	////current_time = ros::Time::now();		
	//first, we'll publish the transform over tf
	////tf::StampedTransform transform_imu;

	//Transformation necessary to the robot_pose_ekf node
	////transform_imu.stamp_=current_time;
	////transform_imu.frame_id_="base_footprint";
	////transform_imu.child_frame_id_ = "/imu_frame";        
	////tf::Vector3 transl_imu(0,0,0);
	////transl_imu[0]=0; 
	////transl_imu[1]=0; 
	////transl_imu[2]=0;
	////transform_imu.setOrigin(transl_imu);

	////tf::Quaternion odom_quat_imu = tf::createQuaternionFromRPY(0,0,0);
	////transform_imu.setRotation(odom_quat_imu);
	//Publish tf
	////Iodom_broadcaster.sendTransform(transform_imu);
	//***********************END TF TRANSFORM FOR IMU*******************
	//***********************TF TRANSFORM FOR VO************************
	//first, we'll publish the transform over tf
	////tf::StampedTransform transform_vo;

	//current_time = ros::Time::now(); //advance timestamp for vo by a little bit
	//Transformation necessary to the robot_pose_ekf node
	////transform_vo.stamp_=current_time;
	////transform_vo.frame_id_="base_footprint";
	////transform_vo.child_frame_id_ = "/odom";        
	////tf::Vector3 transl_vo(0,0,0);
	////transl_vo[0]=0; 
	////transl_vo[1]=0; 
	////transl_vo[2]=0;
	////transform_vo.setOrigin(transl_vo);

	geometry_msgs::Vector3 currentVelocity;
	currentVelocity.x=0.0;
	currentVelocity.y=0.0;
	currentVelocity.z=0.0;

	tf::Quaternion odom_quat_vo = tf::createQuaternionFromRPY(0,0,0);
	////transform_vo.setRotation(odom_quat_vo);
	//Publish tf
	////odom_broadcaster.sendTransform(transform_vo);
	//***********************END TF TRANSFORM FOR VO***********************
	int imu_obs_index=0;//closest imu obs index to cam obs index 

	while ((proc_obs_no-1 < obs_no) || ((proc_obs_no-1 == -1) && (obs_no == -1)) )
	{		
		//cout<<"\nproc_obs_no:"<<proc_obs_no<<" obs_no:"<<obs_no<<"\n";		
		ransac_stat rs_obj;		
		//if ((obs_no > 0) && (obs_no > proc_obs_no))
		if ((obs_no > 0) && (obs_no+1 > proc_obs_no+OBS_JUMP))
		{									
			//******************** Replace rot by IMU *************************
			//cout<<"\nproc_obs_no:"<<proc_obs_no<<"stacked_q_imu->size()-1:"<<stacked_q_imu->size()-1<<"\n"; 
			pthread_mutex_lock(&stacks_mut_);
				if (proc_obs_no <= stacked_q_imu->size()-1)
				{
					imu_obs_index=get_closest_imu(stacked_cam_stamps->at(proc_obs_no),imu_obs_index);
					//cout<<"\n proc_obs_no:"<<proc_obs_no<<" imu_obs_index:"<<imu_obs_index<<"\n"; 
					temp_q=stacked_q_imu->at(imu_obs_index);
					temp_acc=stacked_acc_imu->at(imu_obs_index);
					//temp_acc=get_gcomp_acc(temp_acc,temp_q);
					current_time=stacked_imu_stamps->at(imu_obs_index);
				}
			pthread_mutex_unlock(&stacks_mut_);
			proc_obs_no=proc_obs_no+OBS_JUMP;
			//*****************************************************************	
			//******************** Publish IMU ********************************
	  		//next, we'll publish the IMU message over ROS
			//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(yaw,-pitch,-roll);

	    		////sensor_msgs::Imu Iodom;
	    		////Iodom.header.stamp = current_time;
			////Iodom.header.frame_id = "/imu_frame";	
			//************* Switch VO quat from IMU quat *******************
			quaternionMsgToTF(temp_q, tf_quat_imu);
			tf::Matrix3x3(tf_quat_imu).getEulerYPR(yaw_imu,pitch_imu,roll_imu); //take care of seq of execution	
			//Sanity check
			//cout<<"\ninput q (w x y z): ("<<temp_q.w<<" "<<temp_q.x<<" "<<temp_q.y<<" "<<temp_q.z<<")\n";
			//geometry_msgs::Quaternion ch_q=tf::createQuaternionMsgFromRollPitchYaw(roll_imu,pitch_imu,yaw_imu);
			//cout<<"\noutput q (w x y z): ("<<ch_q.w<<" "<<ch_q.x<<" "<<ch_q.y<<" "<<ch_q.z<<")\n";
			//end sanity check

			if (once == true)
			{
				lpitch_imu=pitch_imu;
				lyaw_imu=yaw_imu;
				lroll_imu=roll_imu;
				once = false;
			}	
			
			tf::Matrix3x3 rotMat_imu;
			//rotMat_imu.setEulerYPR(yaw_imu-lyaw_imu,-roll_imu+lroll_imu,-pitch_imu+lpitch_imu);
			//r-y-p works very well for no sign change//pyr//ryp fails completely with no sign change//r-yp works barely OK with no sign change//-r-y-p works well with no sign change//-r-yp works barely OK with no sign change//-ry-p fails with no sign change//-p-y-r fails with no sign change
			
			double yaw_diff;
			if ((yaw_imu-lyaw_imu) > 2.0)
			{
				yaw_diff=0.05; //just an avg value
				cout<<">2.0yaw diff encountered";
			}else if ((yaw_imu-lyaw_imu) < -2.0)
			{
				yaw_diff=-0.05; //just an avg value
				cout<<">2.0yaw diff encountered";
			}else
			{
				yaw_diff=yaw_imu-lyaw_imu;
			}
			 
			//rotMat_imu.setEulerYPR(-pitch_imu+lpitch_imu,yaw_diff,-roll_imu+lroll_imu); //-py-r //switching these can cause monotonously increasing path
			rotMat_imu.setEulerYPR(roll_imu-lroll_imu,yaw_diff,-pitch_imu+lpitch_imu); //-py-r //switching these can cause monotonously increasing path
			//ryp has worked very well as well.			
			//-py-r works very well. By far the best
			//-pyr works but translations are shrunk in one dimension, 5 m off in y
			//py-r works ok
			//pyr works well, 5 m off in y
			//ry-p works but not as well
			//ryp works well, 2 m off in y Shrunk in one dimension (turns are very accurate)
			//-ry-p works very well, (turns are very accurate).
			//y-r-p//pry//ypr//y-pr//r-yp//p-yr failed
			geometry_msgs::Quaternion gm_q=tf::createQuaternionMsgFromRollPitchYaw(roll_imu-lroll_imu,pitch_imu-lpitch_imu,yaw_imu-lyaw_imu);
			//********** Convert TFMatrix to OpenCV Mat via EigMat ********
			Eigen::Matrix3d rotEig;	
			tf::matrixTFToEigen(rotMat_imu, rotEig);
			for (int i=0;i<3;i++)
				for (int j=0;j<3;j++)
				{
					imu_rot.at<float>(i,j)=rotEig(i,j);
				}
			//**************************************************************
			////Iodom.orientation=gm_q;
			////Iodom.orientation_covariance[0]= 0.000001;
			////Iodom.orientation_covariance[4]= 0.000001; 
			////Iodom.orientation_covariance[8]= 0.000001;
			////Iodom.angular_velocity_covariance[0]= 0.000304617;
			////Iodom.angular_velocity_covariance[4]= 0.000304617;
			////Iodom.angular_velocity_covariance[8]= 0.000304617;
			////Iodom.linear_acceleration_covariance[0]= 0.0000004;
			////Iodom.linear_acceleration_covariance[4]= 0.0000004;
			////Iodom.linear_acceleration_covariance[8]= 0.0000004;

			//publish the message
			////imu_pub.publish(Iodom);
			//******************* Calculate VO *****************************								
			//ROS_INFO("ekf_subscriber.cpp: Before vodom for frame no. %d",proc_obs_no);
			while(vo_is_ready == false)
			{}
			rs_obj=vodometery_matlab(xs.at(proc_obs_no-OBS_JUMP), ys.at(proc_obs_no-OBS_JUMP), zs.at(proc_obs_no-OBS_JUMP), xs.at(proc_obs_no), ys.at(proc_obs_no), zs.at(proc_obs_no), I_imgs.at(proc_obs_no-OBS_JUMP), I_imgs.at(proc_obs_no), state, ep, proc_obs_no, imu_rot);
			//ROS_INFO("ekf_subscriber.cpp: After vodom for frame no. %d",proc_obs_no);
			//******************** Publish VO *******************************	
	   		//next, we'll publish the odometry message over ROS
			////nav_msgs::Odometry odom;
	    		////odom.header.stamp = current_time;
			////odom.header.frame_id = "/odom";		

			//set the position
			////odom.pose.pose.position.x = rs_obj.transMat.at<float>(0,0);
			////odom.pose.pose.position.y = rs_obj.transMat.at<float>(1,0);
			////odom.pose.pose.position.z = rs_obj.transMat.at<float>(2,0);
			////geometry_msgs::Quaternion odom_quat;
			
			////tf::Quaternion tfq=tf::Quaternion(rs_obj.q.at<float>(1,0), rs_obj.q.at<float>(2,0), rs_obj.q.at<float>(3,0), rs_obj.q.at<float>(0,0));		
			////tf::quaternionTFToMsg (tfq, odom_quat);
			////odom.pose.pose.orientation = odom_quat;
			////tf::Matrix3x3(tfq).getEulerYPR(yaw_cam,pitch_cam,roll_cam); //take care of seq of execution
			
			//compute translation via imu
			//if (0)			
			if ((USE_IMU_TRANS) && (current_time > last_time) && ( (abs(-pitch_imu+lpitch_imu)>IMUTRANS_ANG_THRESH) || (abs(yaw_imu-lyaw_imu)>IMUTRANS_ANG_THRESH) || (abs(-roll_imu+lroll_imu)>IMUTRANS_ANG_THRESH) ) )
			{	
				//Time since last update
				double dt = (current_time - last_time).toSec();
				cout<<"\n"<<dt<<"\n";								

				temp_acc.x=temp_acc.x*dt;
				temp_acc.y=temp_acc.y*dt;
				temp_acc.z=temp_acc.z*dt;
				//Get velocity
				currentVelocity.x=temp_acc.x; //m/s				
				currentVelocity.y=temp_acc.y; //m/s				
				currentVelocity.z=temp_acc.z; //m/s				

				//cout<<"\nInstVelocity.x:"<<temp_acc.x<<"\n";
				//cout<<"\nInstVelocity.y:"<<temp_acc.y<<"\n";
				//cout<<"\nInstVelocity.z:"<<temp_acc.z<<"\n";

				cout<<"\ncurrentVelocity.x:"<<currentVelocity.x<<"\n";
				cout<<"\ncurrentVelocity.y:"<<currentVelocity.y<<"\n";
				cout<<"\ncurrentVelocity.z:"<<currentVelocity.z<<"\n";
				cv::Mat v=cv::Mat(3,1,CV_32FC1);
				v.at<float>(0,0)=currentVelocity.x;
				v.at<float>(1,0)=currentVelocity.y;
				v.at<float>(2,0)=currentVelocity.z;

				//cv::Mat vout=imu_rot*v;
				rs_obj.transMat.at<float>(0,0)=-v.at<float>(1,0)*dt;;
				rs_obj.transMat.at<float>(1,0)=-v.at<float>(2,0)*dt;
				rs_obj.transMat.at<float>(2,0)=v.at<float>(0,0)*dt;
				
			}else if ((current_time > last_time) && (stacked_vo_cam->size() >= 2))
			{
				//Time since last update
				double dt = (current_time - last_time).toSec();							

				temp_acc.x=temp_acc.x*dt;
				temp_acc.y=temp_acc.y*dt;
				temp_acc.z=temp_acc.z*dt;
				//Get velocity
				currentVelocity.x=temp_acc.x; //m/s				
				currentVelocity.y=temp_acc.y; //m/s				
				currentVelocity.z=temp_acc.z; //m/s				

				//Velocity at last instance
				//cv::Mat v=cv::Mat(3,1,CV_32FC1);
				//Mat inv_trans=stacked_vo_cam->at(stacked_vo_cam->size()-1).inv();
				//Mat inv_trans_last=stacked_vo_cam->at(stacked_vo_cam->size()-2).inv();
				//v.at<float>(0,0)=(inv_trans.at<float>(0,3)-inv_trans_last.at<float>(0,3))/dt2;
				//v.at<float>(1,0)=(inv_trans.at<float>(1,3)-inv_trans_last.at<float>(1,3))/dt2;
				//v.at<float>(2,0)=(inv_trans.at<float>(2,3)-inv_trans_last.at<float>(2,3))/dt2;
				//currentVelocity.x=v.at<float>(0,0); //m/s				
				//currentVelocity.y=v.at<float>(1,0); //m/s				
				//currentVelocity.z=v.at<float>(2,0); //m/s	
			}
			//set the velocity
			//odom.twist.twist.linear.x = 0.1; //TBD: calculated via dist/time or s=vt formula
			//odom.twist.twist.linear.y = 0.1;
			//odom.twist.twist.angular.z = 1.0;
			
			// covariance
			////for (unsigned int i=0; i<6; i++)
				////for (unsigned int j=0; j<6; j++)
					////if (i == j)
						////odom.pose.covariance[6*i+j] = pow(0.00017,2);
	
			//publish the message
			////vo_pub.publish(odom);
			//***************************************************************************************
			//Sanity check quaternion to rot and back
			//cout<<"\nIMU q (w x y z): ("<<temp_q.w<<" "<<temp_q.x<<" "<<temp_q.y<<" "<<temp_q.z<<")\n";
			//cout<<"\n imu_rot:\n";
			//DISPMAT(imu_rot);

			//cout<<"\nVO rot:\n";
			//DISPMAT(rs_obj.rotMat);
			//cv::Mat q_chk=r2q(rs_obj.rotMat);
			//cout<<"\n q_chk (0 1 2 3): ("<<q_chk.at<float>(0,0)<<" "<<q_chk.at<float>(1,0)<<" "<<q_chk.at<float>(2,0)<<" "<<q_chk.at<float>(3,0)<<")\n";
			//cv::Mat rot_chk=q2r(q_chk);
			//cout<<"\n VO rot_chk:\n";
			//DISPMAT(rot_chk);

			//end sanity check

			//********************** Update logged poses ***********************
			//update_vo(stacked_vo_cam,imu_rot,rs_obj.transMat);
			update_vo(stacked_vo_cam,rs_obj.rotMat,rs_obj.transMat);
			//cout<<"\nimu roll:"<<-pitch_imu+lpitch_imu<<" imu_pitch:"<<yaw_imu-lyaw_imu<<" imu_yaw:"<<-roll_imu+lroll_imu<<"\n";
			update_rpy(stacked_rpy_imu,-roll_imu+lroll_imu,-pitch_imu+lpitch_imu,yaw_imu-lyaw_imu);
			update_rpy(stacked_rpy_cam,roll_cam,pitch_cam,yaw_cam);	
			//************* Image maniuplation and display *********************
			tf::Vector3 rpy=stacked_rpy_imu->at(stacked_rpy_imu->size()-1);
			dbg_rpy[0]+=rpy[0];
			dbg_rpy[1]+=rpy[1];
			dbg_rpy[2]+=rpy[2];
			std::string rpy_str1= "R:" + std::to_string(dbg_rpy[0]);
			std::string rpy_str2= "P:" + std::to_string(dbg_rpy[1]);
			std::string rpy_str3= "Y:" + std::to_string(dbg_rpy[2]);			
			//std::string rpy_str4= "Vx:" + std::to_string(-currentVelocity.y);
			//std::string rpy_str5= "Vy:" + std::to_string(-currentVelocity.z);
			//std::string rpy_str6= "Vz:" + std::to_string(currentVelocity.x);
			std::string rpy_str7= "Tot Matches:" + std::to_string(rs_obj.match.size());
			std::string rpy_str8= "Good Matches:" + std::to_string(rs_obj.GoodFrames1.size());			
			cv::putText(I_imgs.at(proc_obs_no), rpy_str1, cv::Point(5, 10), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255,255,255), 1.0);
			cv::putText(I_imgs.at(proc_obs_no), rpy_str2, cv::Point(5, 20), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255,255,255), 1.0);				
			cv::putText(I_imgs.at(proc_obs_no), rpy_str3, cv::Point(5, 30), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255,255,255), 1.0);		
			//cv::putText(I_imgs.at(proc_obs_no), rpy_str4, cv::Point(5, 40), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255,255,255), 1.0);		
			//cv::putText(I_imgs.at(proc_obs_no), rpy_str5, cv::Point(5, 50), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255,255,255), 1.0);		
			//cv::putText(I_imgs.at(proc_obs_no), rpy_str6, cv::Point(5, 60), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255,255,255), 1.0);
			cv::putText(I_imgs.at(proc_obs_no), rpy_str7, cv::Point(5, 40), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255,255,255), 1.0);		
			cv::putText(I_imgs.at(proc_obs_no), rpy_str8, cv::Point(5, 50), FONT_HERSHEY_PLAIN, 0.85, CV_RGB(255,255,255), 1.0);		
			cv::Mat output;
			cv::drawKeypoints(I_imgs.at(proc_obs_no),rs_obj.GoodFrames1,output);		
			cv::imshow("intensity window", output);   // Show our image inside it.
			//cv::imshow("intensity window", I_imgs.at(proc_obs_no));   // Show our image inside it.
			//************* End Image display and manipulation ****************
			//************ Last var maintainence ****************
			last_time=current_time;
			lpitch_imu=pitch_imu;
			lyaw_imu=yaw_imu;
			lroll_imu=roll_imu;
			//************* END last var maintenance *******************
		}else
		{	
			//pthread_mutex_lock(&vo_mut_);
			//	vo_is_ready = false;			
			//pthread_mutex_unlock(&vo_mut_);
		}

		ros::spinOnce();
		//usleep(10000); // sleep 10 ms
		loop_rate.sleep();
		key=cvWaitKey(20);
		if (key == 'q') 
		{
			break;
		}
	}
	cvDestroyWindow("intensity window");
	
	std::string ds_path(DATASET_PATH);
	save_stacked_rpy(*(stacked_rpy_imu), ds_path + "IMU_quat+VO_trans/rpyimu.csv");
	save_stacked_rpy(*(stacked_rpy_cam), ds_path + "IMU_quat+VO_trans/rpycam.csv");
	save_stacked_vo_kitti(*(stacked_vo_cam), ds_path + "KITTI/results/imuvo/data/11.txt");

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
	}
}
//*****************************************************************************************************************
void sr_timeCB(const std_msgs::Header::ConstPtr& sr_time)
{
	//ROS_INFO("I receive sr4000 time data");	
	{
		pthread_mutex_lock(&srtim_mut_); 
			cam_stamp=sr_time->stamp;
			srtim_is_ready = true;
		pthread_mutex_unlock(&srtim_mut_);
	}
}
//******************************************************************************************************************
// get a new intensity image and depth image
bool getUpdateImgnCld(cv::Mat& i_img, cv::Mat& d_img, cloudPtr& pc, ros::Time& local_cam_stamp)
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
		pthread_mutex_lock(&srtim_mut_); 
			local_cam_stamp=cam_stamp;
			srtim_is_ready = false;
		pthread_mutex_unlock(&srtim_mut_);
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
					pt.x = -ox; 
					pt.y = -oy; 
					pt.z = oz - 0.01;
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
bool getUpdateIMU(geometry_msgs::Quaternion& local_quat, geometry_msgs::Vector3& local_acc, ros::Time& local_stamp)
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
				local_acc=acc_;
				local_stamp=imu_stamp;
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
	stacked_q_imu= new std::vector<geometry_msgs::Quaternion>();
	stacked_acc_imu= new std::vector<geometry_msgs::Vector3>();
	stacked_imu_stamps= new std::vector<ros::Time>();
	stacked_cam_stamps= new std::vector<ros::Time>();
	
	// subscribe /sw_array 
	ros::Subscriber sr_tim_sub = n.subscribe<std_msgs::Header>("/sr_time", 500, sr_timeCB);
	// subscribe /sw_array 
	ros::Subscriber sr_array_sub = n.subscribe<std_msgs::UInt8MultiArray>("/sr_array", 500, sr_arrayCB_pcl);
	// subscribe /imu_data 
	ros::Subscriber imu_data_sub = n.subscribe<sensor_msgs::Imu>("/imu_vo", 500, imu_dataCB); //using remapped rosbag. use /imu_data for unremapped rosbad (28 Sep 2016)

	// spawn another thread
	pthread_t thread_vo;
	ros::Time local_stamp;
	ros::Time local_cam_stamp;

	int rc1;
	/*Create independent threads each of which will execute functionC */
	if( (rc1=pthread_create( &thread_vo, NULL, &vo_calc, NULL)) )
   	{
		printf("Thread creation failed: %d\n", rc1);
	}

	geometry_msgs::Quaternion local_quat;
	geometry_msgs::Vector3 local_acc;
	
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
		// LOGD<<"Can't start MATLAB engine in featurehandler.cpp";
                ROS_ERROR("Can't start MATLAB engine in featurehandler.cpp"); 
		exit(-1);
	}
        ROS_WARN("succeed to open matlab");

	while((n.ok()) && (quit == false))
	{
		/*if(getUpdateCloud(cloud))
		{
			//ROS_INFO("sr_subscriber.cpp: great I get a new cloud, show it!");
			//cin.get();
		}*/

		if(getUpdateIMU(local_quat, local_acc, local_stamp))
		{
			//ROS_INFO("sr_subscriber.cpp: great I get a new IMU data, show it!");
			//cin.get();
			pthread_mutex_lock(&stacks_mut_);
				stacked_q_imu->push_back(local_quat);
				stacked_acc_imu->push_back(local_acc);
				stacked_imu_stamps->push_back(local_stamp);
			pthread_mutex_unlock(&stacks_mut_);
		}
		
		if (getUpdateImgnCld(I_img1, D_img1, cloud1, local_cam_stamp)) 
		{
			//cout<<"\n***"<<cloud1->points.size ()<<"***\n";
			//cout<<"\n***"<<cloud1->height<<"x"<<cloud1->width<<"***\n";
			//cin.get();
			pthread_mutex_lock(&stacks_mut_);
				stacked_cam_stamps->push_back(local_cam_stamp);
			pthread_mutex_unlock(&stacks_mut_);

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
				obs_no++;
			pthread_mutex_lock(&vo_mut_);
				vo_is_ready=true;			
			pthread_mutex_unlock(&vo_mut_); 	
		}

		ros::spinOnce();
		usleep(10000); // sleep 10 ms
	}
	engClose(ep);
	pthread_join(thread_vo, NULL);
}
//*******************************************************************************************************************
int main(int argc, char** argv)
{
	ros::init(argc, argv, "ekf_subscriber");

	ros::NodeHandle n;
	pthread_mutex_init(&quit_mut_, NULL);
	pthread_mutex_init(&stacks_mut_, NULL);
	pthread_mutex_init(&g_mut_, NULL);
	pthread_mutex_init(&quat_mut_, NULL);
	pthread_mutex_init(&vo_mut_, NULL);
	pthread_mutex_init(&srtim_mut_, NULL);
	start_loop(n);
	return 0;
}
