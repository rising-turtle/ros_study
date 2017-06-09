//robot_pose_ekf topic /robot_pose_ekf/odom_combined

#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "std_msgs/Int32.h"
//#include "std_msgs/Bool.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/opencv.hpp>

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

using namespace ros;
using namespace cv;
using namespace std;

bool g_cloud_is_ready = false;
bool vo_is_ready = false;
bool quit = false;

pthread_mutex_t vo_mut_;
pthread_mutex_t quit_mut_;

bool g_exit = false; 
bool g_syn_flag = true;//false;
bool g_get_msg = false;
int g_count = 0;
int imu_count = 0;
int obs_no=-1;
int proc_obs_no=0;
int state=0;	

std::vector<cv::Mat> *stacked_vo;
std::vector<cv::Mat> *stacked_vo_covar;
std::vector<geometry_msgs::Quaternion> *stacked_vo_q;

///std::vector<geometry_msgs::Quaternion> *stacked_vo_imu; //for saving 7x1 pose trace

cv::Mat covar;
geometry_msgs::PointStamped p;
geometry_msgs::Quaternion q;

inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}
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
//*******************************************************************************************************************
void save_stacked_covar(std::vector<cv::Mat> stacked_covar, std::string path)
{
	ofstream myfile;
	myfile.open (path);

	for(int k=0;k<stacked_covar.size();k++)
	{
		cv::Mat elem=stacked_covar.at(k);		
		for(int i=0;i<6;i++)
		{
			for(int j=0;j<6;j++)
			{
				myfile<<elem.at<float>(i,j);
				myfile<<",";
			}
		}
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
void save_stacked_rpy(std::vector<geometry_msgs::Quaternion> stacked_q, std::string path)
{
	tf::Quaternion tf_q;
	ofstream myfile;
	myfile.open (path);
	double yaw,pitch,roll;
	tf::Quaternion elem_tf;
	for(int i=0;i<stacked_q.size();i++)
	{
		geometry_msgs::Quaternion elem=stacked_q.at(i);
		quaternionMsgToTF(elem, elem_tf);
		tf::Matrix3x3(elem_tf).getEulerYPR(yaw,pitch,roll);
		myfile<<roll;
		myfile<<",";
		myfile<<pitch;
		myfile<<",";
		myfile<<yaw;
		myfile<<",";
		myfile<<"\n";
	}
	
	myfile.close();
}
//*******************************************************************************************************************
void update_vo(std::vector<cv::Mat> *vo, geometry_msgs::PointStamped p_loc, geometry_msgs::Quaternion q_loc)
{
	//LOGD<<"rot size:"<<rot.rows<<"x"<<rot.cols;
	//LOGD<<"trans size:"<<trans.rows<<"x"<<trans.cols;

	tf::Quaternion tf_q;
	quaternionMsgToTF(q_loc, tf_q);
	tf::Matrix3x3 tf_rotMat;
	tf_rotMat.setRotation(tf_q);

	Eigen::Matrix3d rotEig;	
	tf::matrixTFToEigen(tf_rotMat, rotEig);

	Mat rot=Mat::eye(3,3,CV_32FC1);
	Mat transf_mat=Mat::eye(4,4,CV_32FC1);
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			transf_mat.at<float>(i,j)=rotEig(i,j); //3x3 patch update
			rot.at<float>(i,j)=rotEig(i,j);
		}

	transf_mat.at<float>(0,3)=p_loc.point.x;
	transf_mat.at<float>(1,3)=p_loc.point.y;
	transf_mat.at<float>(2,3)=p_loc.point.z;

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
//******************************************************************************************************************
void vo_dataCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& vo_data)
{
	//ROS_INFO("I receive imu data %d", ++imu_count); 
	// display what I have received, 
  	{
		pthread_mutex_lock(&vo_mut_); 
			//cout<<tf::getYaw(imu_data->orientation);
			//cout<<"\n x:"<<imu_data->orientation.x<<" y:"<<imu_data->orientation.y<<" z:"<<imu_data->orientation.z<<"\n";
			p.header.stamp = vo_data->header.stamp;
			p.point=vo_data->pose.pose.position;
			q = vo_data->pose.pose.orientation;
			//ROS_INFO("rpekf_processor.cpp: 9.");						
			for (unsigned int i=0; i<6; i++)
				for (unsigned int j=0; j<6; j++)
					covar.at<float>(i,j) = vo_data->pose.covariance[6*i+j];
			vo_is_ready = true;
			//ROS_INFO("rpekf_processor.cpp: 10.");						
		pthread_mutex_unlock(&vo_mut_);
	}
}
//*****************************************************************************************************************
void *vo_calc(void *ptr)
{
	ros::NodeHandle n;
	ros::Rate loop_rate(25);

	char key;
	cv::namedWindow("intensity window", cv::WINDOW_AUTOSIZE);// Create a window for display.

	cv::Mat dummy_img=cv::Mat::zeros(180,160,CV_32FC1);
	//ROS_INFO("rpekf_processor.cpp: 1.");
	while ((proc_obs_no-1 < obs_no) || ((proc_obs_no-1 == -1) && (obs_no == -1)) )
	{		
		//ROS_INFO("rpekf_processor.cpp: 1.1.");
		if ((obs_no > 0) && (obs_no > proc_obs_no))
		{						
			proc_obs_no=proc_obs_no+1;
			cv::imshow( "intensity window", dummy_img);   // Show our image inside it.
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
	//ROS_INFO("rpekf_processor.cpp: 2.");
	cvDestroyWindow("intensity window");

	pthread_mutex_lock(&quit_mut_);
		quit = true;
	pthread_mutex_unlock(&quit_mut_);
}
//******************************************************************************************************************
//get vo data
bool getUpdateVO(cv::Mat& covar_, geometry_msgs::PointStamped& p_, geometry_msgs::Quaternion& q_)
{
	try
	{
		if (!vo_is_ready)
		{
			//ROS_INFO("ekf_subscriber.cpp: quat is not ready, wait!");
			return false; 
		}

		{
			//ROS_INFO("rpekf_processor.cpp: 7.");			
			pthread_mutex_lock(&vo_mut_); 
				covar_.create(6,6,CV_32FC1);
				covar.copyTo(covar_);		
				p_=p;
				q_=q;
				obs_no++;				
				vo_is_ready = false;
			pthread_mutex_unlock(&vo_mut_);	
			//ROS_INFO("rpekf_processor.cpp: 8.");			
		}

		return true;
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception trying to display subscribed info: %s", ex.what());
	}
}
//******************************************************************************************************************
//******************************************************************************************************************
void start_loop(ros::NodeHandle& n)
{
	covar=cv::Mat::zeros(6,6,CV_32FC1);
	stacked_vo_covar= new std::vector<cv::Mat>();
	stacked_vo= new std::vector<cv::Mat>();
	stacked_vo_q= new std::vector<geometry_msgs::Quaternion>();
	//stacked_vo_p= new std::vector<geometry_msgs::PointStamped>();	

	// subscribe /robot_pose_ekf
	ros::Subscriber robot_pose_ekf = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined", 500, vo_dataCB); 

	// spawn another thread
	pthread_t thread_vo;
	
	int rc1;
	/*Create independent threads each of which will execute functionC */
	if( (rc1=pthread_create( &thread_vo, NULL, &vo_calc, NULL)) )
   	{
		printf("Thread creation failed: %d\n", rc1);
	}

	cv::Mat covar_loc;
	geometry_msgs::PointStamped p_loc;
	geometry_msgs::Quaternion q_loc;

	while((n.ok()) && (quit == false))
	{
		if(getUpdateVO(covar_loc,p_loc,q_loc))
		{
			//ROS_INFO("rpekf_processor.cpp: 3.");
			//ROS_INFO("sr_subscriber.cpp: great I get a new IMU data, show it!");
			//cin.get();
			stacked_vo_covar->push_back(covar_loc);
			stacked_vo_q->push_back(q_loc);
			update_vo(stacked_vo,p_loc,q_loc);
			//ROS_INFO("rpekf_processor.cpp: 4.");
		}
		ros::spinOnce();
		usleep(10000); // sleep 10 ms
	}
	//ROS_INFO("rpekf_processor.cpp: 5.");
	save_stacked_covar(*(stacked_vo_covar),"/home/emaad22/Desktop/initial result/rosbag2/IMU_quat+VO_trans/vo_covar.csv");
	save_stacked_rpy(*(stacked_vo_q),"/home/emaad22/Desktop/initial result/rosbag2/IMU_quat+VO_trans/rpyfused.csv");
	save_stacked_vo(*(stacked_vo),"/home/emaad22/Desktop/initial result/rosbag2/IMU_quat+VO_trans/vopath_fused.csv");
	//ROS_INFO("rpekf_processor.cpp: 6.");
	pthread_join(thread_vo, NULL);
}
//*******************************************************************************************************************
int main(int argc, char** argv)
{
	ros::init(argc, argv, "rpekf_processor");

	ros::NodeHandle n;
	pthread_mutex_init(&vo_mut_, NULL);
	start_loop(n);
	return 0;
}
