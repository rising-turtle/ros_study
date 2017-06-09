#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include "sr4000handler.cpp"
#include "ransac_stat.h"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"

using namespace cv;
using namespace std;

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;

inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}
//********************************************************************************************************
int choose(int n, int k)
{
	if (k == 0 )
		return 1;
	return ((n * choose(n - 1, k - 1)) / k);
}
//********************************************************************************************************
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
//*******************************************************************************************************
cv::Mat matperElem(cv::Mat in1, cv::Mat in2, std::string oper)
{ 
	Mat out=Mat::zeros(in1.rows, in1.cols, CV_32FC1);
	for (int i = 0; i < in1.rows; i++)
	{
		float* ptrin1 = in1.ptr<float>(i);
		float* ptrin2 = in2.ptr<float>(i);
		float* ptrout = out.ptr<float>(i);
		for (int j = 0; j < in1.cols; j++)
		{
			if (strcmp(oper.c_str(),"add")==0)
				ptrout[j] = (float)ptrin1[j]+(float)ptrin2[j];
			else if (strcmp(oper.c_str(),"sub")==0)
				ptrout[j] = (float)ptrin1[j]-(float)ptrin2[j];
			else if (strcmp(oper.c_str(),"mul")==0)
				ptrout[j] = (float)ptrin1[j]*(float)ptrin2[j];
			else if (strcmp(oper.c_str(),"div")==0)
				ptrout[j] = (float)ptrin1[j]/(float)ptrin2[j];
			else if (strcmp(oper.c_str(),"sqrt")==0)
				ptrout[j] = sqrt((float)ptrin1[j]);
		}
	}
	return out;
}
//***********************************************************************************************************
void find_transform_matrix(std::vector<cv::Point3f> op_pset1, std::vector<cv::Point3f> op_pset2, cv::Mat* rot, cv::Mat* trans, int& state)
{
	//state 0 means no solution, 1 means solution, 2 means coplanar solution
	Mat pset1 = Mat(3,op_pset1.size(),CV_32FC1);//X points x 3 coords
	Mat pset2 = Mat(3,op_pset2.size(),CV_32FC1); //X points x 3 coords

	for (unsigned int i=0; i<op_pset1.size(); i++)  
	{
		pset1.at<float>(0,i)=op_pset1.at(i).x;
		pset1.at<float>(1,i)=op_pset1.at(i).y;
		pset1.at<float>(2,i)=op_pset1.at(i).z;
	}
	//LOGD<<"pset1:";
	//LOGMAT(pset1);

	for (unsigned int i=0; i<op_pset2.size(); i++)  
	{
		pset2.at<float>(0,i)=op_pset2.at(i).x;
		pset2.at<float>(1,i)=op_pset2.at(i).y;
		pset2.at<float>(2,i)=op_pset2.at(i).z;
	}
	//LOGD<<"pset2:";
	//LOGMAT(pset2);

	cv::Mat ct1 = cv::Mat::zeros(3,1,CV_32FC1); //3 points 1 coords
	cv::Mat ct2 = cv::Mat::zeros(3,1,CV_32FC1); //3 points 1 coords
	//trans=cv::Mat::zeros(3,1,CV_32FC1); //3 points 1 coords. Its a 3x1 matrix

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<pset1.rows; j++)
		{
			ct1.at<float>(i,0)=ct1.at<float>(i,0)+pset1.at<float>(i,j);
		}
		ct1.at<float>(i,0)=ct1.at<float>(i,0)/(float)pset1.cols;
	}

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<pset2.rows; j++)
		{
			ct2.at<float>(i,0)=ct2.at<float>(i,0)+pset2.at<float>(i,j);
		}
		ct2.at<float>(i,0)=ct2.at<float>(i,0)/(float)pset2.cols;
	}

	cv::Mat q1 = cv::Mat::zeros(3,1,CV_32FC1); //3 points 1 coords
	cv::Mat q1t = cv::Mat::zeros(1,3,CV_32FC1); //1x3 //q1 transposed
	cv::Mat q2 = cv::Mat::zeros(3,1,CV_32FC1); //3 points 1 coords

	cv::Mat q21= cv::Mat::zeros(3,3,CV_32FC1); //3 points x 3 points // assuming points in both sets are equal
	cv::Mat H_= cv::Mat::zeros(3,3,CV_32FC1); //3 points x 3 points // assuming points in both sets are equal

	for (int k=0; k<pset1.rows; k++) //assuming pset1 and pset2 have same coord dimensions
	{
		for (int l=0; l<3; l++) //assuming pset1 and pset2 have same coord dimensions
		{
			q1.at<float>(l,0)=pset1.at<float>(l,k)-ct1.at<float>(l,0);
			q2.at<float>(l,0)=pset2.at<float>(l,k)-ct2.at<float>(l,0);
		}
		transpose(q1,q1t);
		//cout<<q1.size()<<"\n";
		//cout<<q2t.size(); //3x1
		q21=q2*q1t; //3x3=3x1*1x3
		H_=H_+q21; //
	}
	SVD svd(H_,SVD::FULL_UV);

	Mat UT;
	transpose(svd.u,UT);
	Mat V;
	transpose(svd.vt,V);
	Mat W=svd.w;

	//LOGD<<"U:";
	//LOGMAT(svd.u);


	//LOGD<<"S:";
	//LOGMAT(W);

	//LOGD<<"V:";
	//LOGMAT(V);

	cv::Mat sv=cv::Mat::zeros(3,1,CV_32FC1);

	for (int i=0;i<W.rows;i++)
	{
		sv.at<float>(i,0)=abs(W.at<float>(i,0));
	}

	cv::Mat Xq=V*UT;
	//LOGD<<"Xq:";
	//LOGMAT(Xq);

	double mdet=determinant(Xq);

	//LOGD<<"mdet: "<<mdet;

	double threshold = 0.00000000000001;
	state=0;

	if (cvRound(mdet) == 1)
	{
		for(int ii=0;ii<Xq.rows;ii++)
			for(int jj=0;jj<Xq.cols;jj++)
				rot->at<float>(ii,jj)=Xq.at<float>(ii,jj);
		//LOGD<<"1";
		Mat loc_trans=ct1-(Xq*ct2); //3x1-(3x3*3x1)

		for(int ii=0;ii<loc_trans.rows;ii++)
			for(int jj=0;jj<loc_trans.cols;jj++)
				trans->at<float>(ii,jj)=loc_trans.at<float>(ii,jj);
		state=1;
	}else if (cvRound(mdet) == -1)
	{
		vector<int> zn;
		//LOGD<<"3";
		for(int i=0;i<3;i++)
		{
			if (sv.at<float>(i,0) < threshold)
			{
				zn.push_back(i);
			}
		}
		//LOGD<<"4";
		int zs=zn.size();
		if (zs == 1)
		{
			//LOGD<<"5";
			for(int i=0;i<V.rows;i++)
			{
				V.at<float>(i, zn.at(0))=-V.at<float>(i,zn.at(0));
			}
			//LOGD<<"6";
			Mat loc_rot = V * UT;

			for(int ii=0;ii<loc_rot.rows;ii++)
				for(int jj=0;jj<loc_rot.cols;jj++)
					rot->at<float>(ii,jj)=loc_rot.at<float>(ii,jj);

			Mat loc_trans = ct1 - loc_rot*ct2;

			for(int ii=0;ii<loc_trans.rows;ii++)
				for(int jj=0;jj<loc_trans.cols;jj++)
					trans->at<float>(ii,jj)=loc_trans.at<float>(ii,jj);

			state = 2;
		}else
		{
			state = -1;
			//LOGD<<"9";
			Mat loc_rot=Mat::eye(pset1.rows,pset1.rows,CV_32FC1);
			for(int ii=0;ii<loc_rot.rows;ii++)
				for(int jj=0;jj<loc_rot.cols;jj++)
					rot->at<float>(ii,jj)=loc_rot.at<float>(ii,jj);

			Mat loc_trans=cv::Mat::zeros(3,1,CV_32FC1);
			for(int ii=0;ii<loc_trans.rows;ii++)
				for(int jj=0;jj<loc_trans.cols;jj++)
					trans->at<float>(ii,jj)=loc_trans.at<float>(ii,jj);
		}
	}else
	{
		state = -1;
		Mat loc_rot=Mat::eye(pset1.rows,pset1.rows,CV_32FC1);
		for(int ii=0;ii<loc_rot.rows;ii++)
			for(int jj=0;jj<loc_rot.cols;jj++)
				rot->at<float>(ii,jj)=loc_rot.at<float>(ii,jj);

		Mat loc_trans=cv::Mat::zeros(3,1,CV_32FC1);
		for(int ii=0;ii<loc_trans.rows;ii++)
			for(int jj=0;jj<loc_trans.cols;jj++)
				trans->at<float>(ii,jj)=loc_trans.at<float>(ii,jj);
	}
}
//********************************************************************************************************

cv::Mat getSift(cv::Mat img,std::vector<cv::KeyPoint> *rawf)
{
	cv::Mat descp;
	cv::vector<cv::KeyPoint> rawframe;
	int featNos=50; //The number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast)
	int S = 3;//# of scale levels within each octave
	float edgeThresh = 10.0; //10.0//Decrease to get more feats. Feature which have flatness score above this threshold are ignored. Bigger values accept more features.
	float contThresh =0.04*(1.0/(double)(S*2.0)); //0.05 default, Increase to get more feats 
	//LOGD<<"Current contThresh:"<<contThresh<<" old contThresh:"<<0.04/S/2.0<<" OpenCV default:"<<0.05;
	//float magnif = 3.0; //Descriptor window magnification
	float sigma0=pow((1.6*2.0),1.0/(double)S); //Smoothing of the level 0 of octave 0 of the scale space. By default it is set to be equivalent to the value 1.6 of [1]. Since however 1.6 is the smoothing of the level -1 and Simga0 of the level 0, the actual value is NOT 1.6.
	
	// SIFT feature detector and feature extractor
	// initialize detector and extractor
	cv::SiftFeatureDetector detector(featNos, /*nFeatures*/ S, /*nOctaveLayers*/ contThresh, /*contrastThreshold*/ edgeThresh, /*edgeThreshold*/ sigma0);
	//cv::SiftDescriptorExtractor extractor(magnif);
	cv::SiftDescriptorExtractor extractor;

	//Mat img_;
	//img_ = imread( "2qo.png", CV_LOAD_IMAGE_GRAYSCALE );
	// Feature detection
	detector.detect(img, rawframe);

	extractor.compute(img, rawframe, descp);
	//imwrite("denimg.png", img);
	
	// Add results to image and save.
	//cv::Mat output;
	//cv::drawKeypoints(img, rawframe, output);
	//cv::imwrite("sift_result.jpg", output);

	// Feature descriptor computation
	//cout<<"\n"<<(int)rawframe.size()<<" keypoints are found.\n" ;

	for(unsigned int i=0;i<rawframe.size();i++)
	{
		//tt.x=rawframe.at(i).pt.x; // bw 0 - 176
		//tt.y=rawframe.at(i).pt.y; //bw 0 - 144	
		rawf->push_back(rawframe.at(i));
	}

	return descp;
}
//********************************************************************************************************
cv::Mat remMatrow(cv::Mat matIn, int row)
{
	//row // Row to delete.
	// Removing a row
	//matIn	// Matrix of which a row will be deleted.
	cv::Mat matOut=Mat(matIn.rows-1,matIn.cols,CV_32FC1);   // Result: matIn less that one row.

	if ( row > 0 ) // Copy everything above that one row.
	{
		cv::Rect rect( 0, 0, matIn.cols, row );
		matIn(rect).copyTo(matOut(rect));
	}

	if ( row < matIn.rows - 1 ) // Copy everything below that one row.
	{
		cv::Rect rect1( 0, row + 1, matIn.cols, matIn.rows - row - 1 );
		cv::Rect rect2( 0, row, matIn.cols, matIn.rows - row - 1 );
		matIn( rect1 ).copyTo( matOut( rect2 ) );
	}
	return matOut;	
}
//********************************************************************************************************
cv::Mat confidenceFilter(cv::Mat descp, cv::Mat c_map, std::vector<cv::KeyPoint> *kps)
{
	//TBD: Need to use GFOR
	float max_val=-10000.0;
	for (int i=0;i<c_map.rows;i++)
		for (int j=0;j<c_map.cols;j++)
			if (c_map.at<float>(i,j) > max_val)
				max_val=c_map.at<float>(i,j);

	float thresh=0.8; //this is the precentage of max confidence value found in confidence map

	vector<int> idx2rem;
	
	unsigned int i=0;
	while(i<kps->size())
	{
		int row=kps->at(i).pt.y; //x and y does not need to be switched
		int col=kps->at(i).pt.x;
		if (c_map.at<float>(row,col) < (thresh*max_val))
		{
			idx2rem.push_back(i);
		}
		i++;
	}

	for (unsigned int i=0;i<idx2rem.size();i++)
	{
		Mat temp;
		temp=remMatrow(descp,idx2rem.at(i)); 
		descp.create(temp.rows,temp.cols,descp.type());
		temp.copyTo(descp);

		kps->erase(kps->begin()+idx2rem.at(i));

		for(unsigned int kk=0;kk<idx2rem.size();kk++) //since list is changing size at each iteration thus indexes do not remain valid
			idx2rem.at(kk)=idx2rem.at(kk)-1;
	}
	
	cv::Mat out;
	descp.copyTo(out);
	return out;
}
//********************************************************************************************************
void do_ransac(std::vector<cv::KeyPoint> *rawframe1, std::vector<cv::KeyPoint> *rawframe2, std::vector<cv::DMatch> good_matches,sr4000data step_n_1, sr4000data step_n,int &cnum,std::vector<cv::DMatch> *nMatch) //matlab function: ransac_dr_ye
{
	int pnum=good_matches.size();

	if (pnum < 4)
	{
		cnum=-1;
		//LOGD<<"featurehandler::do_ransac: Number of points is smaller than 4. Insufficient for ransac";
		return;
	}
	
	int rawframe1_idx,rawframe2_idx, col1, col2, row1, row2;
	cv::KeyPoint matched_pix1;
	cv::KeyPoint matched_pix2;
	Mat pset1_mat;//Mat(3,1,CV_32FC1); //eventually 3 x pnum
	Mat pset2_mat;//Mat(3,1,CV_32FC1); //eventually 3 x pnum

	for (int i=0;i<pnum;i++)
	{
		rawframe1_idx=good_matches.at(i).queryIdx; //here x is query index
		rawframe2_idx=good_matches.at(i).trainIdx; //here y is database index
		matched_pix1=rawframe1->at(rawframe1_idx);
		matched_pix2=rawframe2->at(rawframe2_idx);
		
		col1 = cvRound(matched_pix1.pt.x);
		row1 = cvRound(matched_pix1.pt.y);
		col2 = cvRound(matched_pix2.pt.x);
		row2 = cvRound(matched_pix2.pt.y);

		cv::Point3f pt1,pt2;
		pt1.x= -step_n_1.x.at<float>(row1,col1);
		pt1.y= -step_n_1.y.at<float>(row1,col1);
		pt1.z= step_n_1.z.at<float>(row1,col1);

		Mat temp31=(Mat_<float>(3,1) << pt1.x, pt1.y, pt1.z); 
		if (pset1_mat.empty() == true)
			temp31.copyTo(pset1_mat);
		else
			cv::hconcat(pset1_mat,temp31,pset1_mat);

		pt2.x= -step_n.x.at<float>(row2,col2);
		pt2.y= -step_n.y.at<float>(row2,col2);
		pt2.z= step_n.z.at<float>(row2,col2);

		Mat temp31_2=(Mat_<float>(3,1) << pt2.x, pt2.y, pt2.z); 
		if (pset2_mat.empty() == true)
			temp31_2.copyTo(pset2_mat);
		else
			cv::hconcat(pset2_mat,temp31_2,pset2_mat);			
	}

	Mat sq1=matperElem(pset2_mat.row(0),pset2_mat.row(0), "mul"); //1 x pnum
	Mat sq2=matperElem(pset2_mat.row(1),pset2_mat.row(1), "mul"); //1 x pnum
	Mat sq3=matperElem(pset2_mat.row(2),pset2_mat.row(2), "mul"); //1 x pnum

	Mat dummy=Mat::zeros(1,1,CV_32FC1);
	Mat norm_pset2=matperElem(sq1+sq2+sq3,dummy,"sqrt"); //1 x pnum

	float minZ=100000.0;
	int pmZ=0; //col index of minimum value
	for (int i=0;i<pset2_mat.cols;i++)
	{
		if ((norm_pset2.at<float>(0,i) > 0.4) && (minZ > pset2_mat.at<float>(2,i))) 
		{
			minZ=pset2_mat.at<float>(2,i);
			pmZ=i;
		}
	}

	float term1=pset2_mat.at<float>(0,pmZ)*pset2_mat.at<float>(0,pmZ);
	float term2=pset2_mat.at<float>(1,pmZ)*pset2_mat.at<float>(1,pmZ);
	float term3=pset2_mat.at<float>(2,pmZ)*pset2_mat.at<float>(2,pmZ);;
	
	float dist=sqrt(term1+term2+term3);
	int num_rs[4];
	int ns=4;
	num_rs[0]=cvRound((float)(pnum-1)*(float(rand()%100)/100.0)); 
	num_rs[1]=cvRound((float)(pnum-1)*(float(rand()%100)/100.0)); 
	num_rs[2]=cvRound((float)(pnum-1)*(float(rand()%100)/100.0)); 
	num_rs[3]=cvRound((float)(pnum-1)*(float(rand()%100)/100.0));		

	bool ind_dup1,ind_dup2,ind_dup3;
	if ((good_matches.at(num_rs[0]).queryIdx == good_matches.at(num_rs[1]).queryIdx) || (good_matches.at(num_rs[0]).trainIdx == good_matches.at(num_rs[1]).trainIdx) )
		ind_dup1=true;
	else
		ind_dup1=false;

	if ((good_matches.at(num_rs[0]).queryIdx == good_matches.at(num_rs[2]).queryIdx) || (good_matches.at(num_rs[1]).trainIdx == good_matches.at(num_rs[2]).queryIdx) || (good_matches.at(num_rs[0]).trainIdx == good_matches.at(num_rs[2]).trainIdx) || (good_matches.at(num_rs[1]).trainIdx == good_matches.at(num_rs[2]).trainIdx) )
		ind_dup2=true;
	else
		ind_dup2=false;

	if ((good_matches.at(num_rs[0]).queryIdx == good_matches.at(num_rs[3]).queryIdx) || (good_matches.at(num_rs[1]).queryIdx == good_matches.at(num_rs[3]).trainIdx) || (good_matches.at(num_rs[2]).queryIdx == good_matches.at(num_rs[3]).queryIdx) || (good_matches.at(num_rs[0]).trainIdx == good_matches.at(num_rs[3]).queryIdx) || (good_matches.at(num_rs[1]).trainIdx == good_matches.at(num_rs[3]).trainIdx) || (good_matches.at(num_rs[2]).trainIdx == good_matches.at(num_rs[3]).trainIdx))
		ind_dup3=true;
	else
		ind_dup3=false;

	int expiry_timer; //usually equal to max number of good features usually detected per frame
	expiry_timer=50;
	while ((num_rs[1] == num_rs[0]) || ind_dup1)
	{
		num_rs[1] = cvRound((float)(pnum-1)*(float(rand()%100)/100.0));
		if ((good_matches.at(num_rs[0]).queryIdx == good_matches.at(num_rs[1]).queryIdx) || (good_matches.at(num_rs[0]).trainIdx == good_matches.at(num_rs[1]).trainIdx) )
			ind_dup1 = true;
		else
			ind_dup1 = false;
		expiry_timer--;
		if (expiry_timer < 0)
		{
			break;
		}
	}

	expiry_timer=50;
	while ((num_rs[2] == num_rs[0]) || (num_rs[2] == num_rs[1]) || ind_dup2)
	{
		num_rs[2] = cvRound((float)(pnum-1)*(float(rand()%100)/100.0));
		if ((good_matches.at(num_rs[0]).queryIdx == good_matches.at(num_rs[2]).queryIdx) || (good_matches.at(num_rs[1]).queryIdx == good_matches.at(num_rs[2]).queryIdx) || (good_matches.at(num_rs[0]).trainIdx == good_matches.at(num_rs[2]).trainIdx) || (good_matches.at(num_rs[1]).trainIdx == good_matches.at(num_rs[2]).trainIdx))
			ind_dup2 = true;
		else
			ind_dup2 = false;

		expiry_timer--;
		if (expiry_timer < 0)
		{
			break;
		}
	}

	expiry_timer=50;
	while ((num_rs[3] == num_rs[0]) || (num_rs[3] == num_rs[1]) || (num_rs[3] == num_rs[2]) || ind_dup3)
	{
		//LOGD<<"num_rs[3]:"<<num_rs[3]<<" num_rs[2]:"<<num_rs[2]<<" num_rs[1]:"<<num_rs[1]<<" num_rs[0]:"<<num_rs[0];
		//LOGD<<"ind_dup3:"<<ind_dup3;
		num_rs[3] = cvRound((float)(pnum-1)*(float(rand()%100)/100.0));

		if ((good_matches.at(num_rs[0]).queryIdx == good_matches.at(num_rs[3]).queryIdx) || (good_matches.at(num_rs[1]).queryIdx == good_matches.at(num_rs[3]).trainIdx) || (good_matches.at(num_rs[2]).queryIdx == good_matches.at(num_rs[3]).queryIdx) || (good_matches.at(num_rs[0]).trainIdx == good_matches.at(num_rs[3]).queryIdx) || (good_matches.at(num_rs[1]).trainIdx == good_matches.at(num_rs[3]).trainIdx) || (good_matches.at(num_rs[2]).trainIdx == good_matches.at(num_rs[3]).trainIdx))
			ind_dup3 = true;
		else
			ind_dup3 = false;

		expiry_timer--;
		if (expiry_timer < 0)
		{
			break;
		}
	}

	vector<cv::Point3f> op_rset1, op_rset2;
	
	for(int i=0; i<ns ;i++)
	{
		rawframe1_idx=good_matches.at(num_rs[i]).queryIdx; //here x is query index
		rawframe2_idx=good_matches.at(num_rs[i]).trainIdx; //here y is database index
		matched_pix1=rawframe1->at(rawframe1_idx);
		matched_pix2=rawframe2->at(rawframe2_idx);
		
		col1 = cvRound(matched_pix1.pt.x);
		row1 = cvRound(matched_pix1.pt.y);
		col2 = cvRound(matched_pix2.pt.x);
		row2 = cvRound(matched_pix2.pt.y);

		cv::Point3f pt1,pt2;
		pt1.x= -step_n_1.x.at<float>(row1,col1);
		pt1.y= -step_n_1.y.at<float>(row1,col1);
		pt1.z= step_n_1.z.at<float>(row1,col1);
		Mat temp31=(Mat_<float>(3,1) << pt1.x, pt1.y, pt1.z); 
		
		op_rset1.push_back(pt1);
		
		pt2.x= -step_n.x.at<float>(row2,col2);
		pt2.y= -step_n.y.at<float>(row2,col2);
		pt2.z= step_n.z.at<float>(row2,col2);
		Mat temp31_2=(Mat_<float>(3,1) << pt2.x, pt2.y, pt2.z); 

		op_rset2.push_back(pt2);
	}

	Mat rotMat=Mat(3,3,CV_32FC1);
	Mat transMat=Mat(3,1,CV_32FC1);
	int state; //0 means no solution, 1 means solution, 2 means coplanar solution

	find_transform_matrix(op_rset1,op_rset2,&rotMat,&transMat,state);

	Mat pset21=rotMat*pset2_mat; // TBD: pset21 is 3 x pnum(size of all matches), pset2_mat must be 3 x pnum by inference

	vector<float> d_diff;

	for(int i=0;i<pnum;i++)
		d_diff.push_back(0);
	
	for(int i=0;i<pnum;i++)
	{
		for(int j=0;j<pset21.rows;j++)
			pset21.at<float>(j,i)= pset21.at<float>(j,i)+transMat.at<float>(j,0); //TBD: transMat must be a 3 x 1 

		for(int j=0;j<3;j++) //TBD: assert pset21 must have 3 rows
		{
			d_diff.at(i) = d_diff.at(i) + pow((pset21.at<float>(j,i)-pset1_mat.at<float>(j,i)),2); 
		}
	}

	vector<int> good_pt;
	for (unsigned int i=0;i<d_diff.size();i++)
	{
		if (d_diff.at(i) < 0.005*dist)
		{
			good_pt.push_back(i);
		}
	}

	cnum=good_pt.size();

	if (cnum == 0)
	{
		
		return;
	}
	
	for (int i=0;i<cnum;i++)
	{
		//matchdef node;
		//node.queryIdx=good_matches.at(good_pt.at(i)).queryIdx;
		DMatch temp_m=good_matches.at(good_pt.at(i));
		nMatch->push_back(temp_m);
	}
}
//*****************************************************************************************
ransac_stat vodometery(sr4000data step_n_1, sr4000data step_n, int &state, int obs_no)
{
	ransac_stat rs_obj;
	std::vector<cv::Point3f> op_pset1;
	std::vector<cv::Point3f> op_pset2;
	
	cv::Mat rotMat=Mat::eye(3,3,CV_32FC1);
	cv::Mat transMat=Mat::zeros(3,1,CV_32FC1);

	Mat descp1_filt;
	Mat descp1; // we HAVE to keep it a matrix since its used in FLANN matcher
	std::vector<cv::KeyPoint> rawframe1;
	descp1=getSift(step_n_1.den_norm_img,&rawframe1); //each row of descp corresponds to each keypoint in rawframe
	rs_obj.nFeatures1=rawframe1.size();
	
	descp1_filt=confidenceFilter(descp1,step_n_1.confidence_map,&rawframe1);
	rs_obj.nF1_Confidence_Filtered=rawframe1.size();

	Mat descp2_filt;
	Mat descp2;
	std::vector<cv::KeyPoint> rawframe2;
	descp2=getSift(step_n.den_norm_img,&rawframe2); //each row of descp corresponds to each keypoint in rawframe
	rs_obj.nFeatures2=rawframe2.size();

	descp2_filt=confidenceFilter(descp2,step_n.confidence_map,&rawframe2);

	rs_obj.nF2_Confidence_Filtered=rawframe2.size();

	// Matching descriptor vectors using FLANN matchrow2 = cvRounder
	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(descp1_filt, descp2_filt, matches);

	double max_dist = -10000; double min_dist = 10000;
	//-- Quick calculation of max and min distances between keypoints
	for(unsigned int i = 0; i < matches.size() ; i++ )
	{
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	std::vector<cv::DMatch> good_matches;
	for(unsigned int i = 0; i < matches.size(); i++ )
	{ 
		if( matches[i].distance <= std::max(1.5*min_dist, 450.0) ) 
		{ 
			cv::DMatch temp_m=matches[i];
			good_matches.push_back(temp_m); 
		}
	}
	//**************************************** for debug only *********************************************
	// Add results to images and save.
	char buff[50];
	cv::Mat output1;
	cv::vector<cv::KeyPoint> rawframe_cv1;
	for(unsigned int ii=0;ii<rawframe1.size();ii++)
	{
		rawframe_cv1.push_back(rawframe1.at(ii));
	}

	cv::Mat output2;
	cv::vector<cv::KeyPoint> rawframe_cv2;
	for(unsigned int ii=0;ii<rawframe2.size();ii++)
	{
		rawframe_cv2.push_back(rawframe2.at(ii));
	}

	cv::drawKeypoints(step_n_1.den_norm_img, rawframe_cv1, output1);
	cv::drawKeypoints(step_n.den_norm_img, rawframe_cv2, output2);
	
	for(int ii=0;ii<50;ii++)
		buff[ii]='\0';

	sprintf(buff,"./IMAGES/img%d_0.jpg",obs_no); 
	cv::imwrite(buff,output1);
	
	for(int ii=0;ii<50;ii++)
		buff[ii]='\0';
	
	sprintf(buff,"./IMAGES/img%d_1.jpg",obs_no); 
	cv::imwrite(buff,output2);

	cv::Mat output3;
	cv::drawMatches(step_n_1.den_norm_img, rawframe_cv1,step_n.den_norm_img, rawframe_cv2,good_matches,output3,Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	for(int ii=0;ii<50;ii++)
		buff[ii]='\0';
	
	sprintf(buff,"./IMAGES/img%d_%d_match.jpg",obs_no,obs_no+1); 
	cv::imwrite(buff,output3);

	//**************************************** for debug only end ********************************************
	rs_obj.nMatches=good_matches.size(); //equal to pnum

	float epsilon=0.01;
	int maxCNUM=0;
	int nSetHypGen=4, rst, nIterations=0;
	vector< vector<cv::DMatch> > tmp_nmatch; //list of nMatch
	int cnum=0, op_num=0;

	
	std::vector<cv::DMatch> op_match; //optimized matches conforming to a particular motion hypothesis

	if (rs_obj.nMatches < 4) 
	{
		//LOGD<<"\nvodometery: too few sift parameters for ransac. RANSAC fails.\n";
		rotMat=Mat::eye(3,3,CV_32FC1);
		transMat=Mat::zeros(3,1,CV_32FC1);
		rs_obj.rotMat=rotMat;
		rs_obj.transMat=transMat;
		rs_obj.q=r2q(rotMat);
		rs_obj.SolutionState=4;
		rs_obj.status=-1;
		return rs_obj;
	}else
	{
		rst=min(700,choose(rs_obj.nMatches,4));	//original 700
		nIterations=rst;
		//LOGD<<"nIterations for RANSAC-vodometery: "<<nIterations;
		int rs_ind;
		for(int i=0;i<min(rst,nIterations);i++)
		{
			//LOGD<<"Pre do_ransac"; 
			std::vector<cv::DMatch> nMatch;
			do_ransac(&rawframe1,&rawframe2,good_matches,step_n_1,step_n,cnum,&nMatch);		
			//LOGD<<"Post do_ransac"; 
			tmp_nmatch.push_back(nMatch); //tmp_nmatch(:,k,i) this is i index being updated

			if (cnum > maxCNUM)
			{
				maxCNUM=cnum;
				rs_ind=i;
				double logep=log(epsilon);
				double frac=1.0-pow(((double)maxCNUM/(double)rs_obj.nMatches),nSetHypGen);
				nIterations=(int)(5.0*ceil(logep/log(frac)));
				//5*ceil(log(epsilon) / log( (1-(maxCNUM/pnum)^nSetHypothesisGenerator) ) );
			}
		}
		
		op_num=maxCNUM;
		
		if (op_num < 3)
		{
			//LOGD<<"\nvodometery: No consensus found, RANSAC fails.\n";
			rotMat=Mat::eye(3,3,CV_32FC1);				
			transMat=Mat::zeros(3,1,CV_32FC1);
			rs_obj.rotMat=rotMat;
			rs_obj.transMat=transMat;
			rs_obj.q=r2q(rotMat);
			rs_obj.SolutionState=4;
			rs_obj.status=-1;
			return rs_obj;
		}

		//LOGD<<"Best ransac hypothesis has this many points: op_num:"<<op_num; //best ransac hypothesis has this many points

		vector<DMatch> tmp=tmp_nmatch.at(rs_ind); //size = op_num
		for (int k=0; k<op_num; k++)
		{		
			DMatch temp_m=tmp.at(k);
			op_match.push_back(temp_m); //optimized match from RANSAC
			//LOGD<<"Matching pt (Query): "<<rawframe1.at(tmp.at(k).queryIdx).x<<","<<rawframe1.at(tmp.at(k).queryIdx).y; //confirm if these keypoints actually match between n-1 and n images via opencv draw keypoints
			//LOGD<<"Matching pt (Train): "<<rawframe2.at(tmp.at(k).trainIdx).x<<","<<rawframe2.at(tmp.at(k).trainIdx).y;
		}
		//LOGD<<"Marker 1";
		//tmp_nmatch; we assume that it will be an array of 2 rowed [rs_obj.nMatches x rst] matrices.
	}

	int rawframe1_idx,rawframe2_idx, col1, col2, row1, row2;
	vector<cv::KeyPoint> GoodRF_1;
	vector<cv::KeyPoint> GoodRF_2;
	Mat GoodDescp1;
	Mat GoodDescp2;

	for (int i=0; i<op_num ;i++)
	{
		rawframe1_idx=op_match.at(i).queryIdx; //order verified//here x is query index //TBD: verify the order of these query and index assignment
		rawframe2_idx=op_match.at(i).trainIdx; //here y is database index
		
		cv::KeyPoint matched_pix1;
		cv::KeyPoint matched_pix2;

		matched_pix1=rawframe1.at(rawframe1_idx);
		matched_pix2=rawframe2.at(rawframe2_idx);

		GoodRF_1.push_back(matched_pix1);
		GoodRF_2.push_back(matched_pix2);

		Mat temp_gd1=descp1_filt.row(rawframe1_idx);
		Mat temp_gd2=descp2_filt.row(rawframe2_idx);

		GoodDescp1.push_back(temp_gd1);
		GoodDescp2.push_back(temp_gd2);

		col1 = cvRound(matched_pix1.pt.x);
		row1 = cvRound(matched_pix1.pt.y);

		col2 = cvRound(matched_pix2.pt.x);
		row2 = cvRound(matched_pix2.pt.y);

		cv::Point3f pt1,pt2;
		pt1.x= -step_n_1.x.at<float>(row1,col1);
		pt1.y= -step_n_1.y.at<float>(row1,col1);
		pt1.z= step_n_1.z.at<float>(row1,col1);
		op_pset1.push_back(pt1);

		pt2.x= -step_n.x.at<float>(row2,col2);
		pt2.y= -step_n.y.at<float>(row2,col2);
		pt2.z= step_n.z.at<float>(row2,col2);
		op_pset2.push_back(pt2);
	}

	//LOGD<<"GoodRF_1.size: "<<GoodRF_1.size();
	//LOGD<<"GoodRF_2.size: "<<GoodRF_2.size();

	//LOGD<<"Marker 2";
	////////////////////////////////////for debug only////////////////////////////////////////////////////////////////////////////
	cv::vector<cv::KeyPoint> gf1;
	for(unsigned int ii=0;ii<rawframe1.size();ii++)
	{
		gf1.push_back(rawframe1.at(ii));
	}
	
	//LOGD<<"Marker 3";
	cv::vector<cv::KeyPoint> gf2;
	for(unsigned int ii=0;ii<rawframe2.size();ii++)
	{
		gf2.push_back(rawframe2.at(ii));
	}
	
	//LOGD<<"Marker 4";
	cv::Mat out_img;
	cv::drawMatches(step_n_1.den_norm_img, gf1,step_n.den_norm_img, gf2,op_match,out_img,Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//char buff[50];
	for(int ii=0;ii<50;ii++)
		buff[ii]='\0';

	sprintf(buff,"./IMAGES/img%d_%d_gmatch.jpg",obs_no,obs_no+1); 
	cv::imwrite(buff,out_img);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//reset pose matrices
	rotMat=Mat::eye(3,3,CV_32FC1);
	transMat=Mat::zeros(3,1,CV_32FC1);	
	find_transform_matrix(op_pset1,op_pset2,&rotMat,&transMat,state);
	
	//LOGD<<"Marker 6";
	rs_obj.GoodFrames1=GoodRF_1;
	rs_obj.GoodFrames2=GoodRF_2;
	rs_obj.GoodDescp1=GoodDescp1;
	rs_obj.GoodDescp2=GoodDescp2;

	rs_obj.op_pset1=op_pset1;
	rs_obj.op_pset2=op_pset2;

	if (state < 1)
	{
		//LOGD<<"\nvodometery: No Solution found.\n";
		rotMat=Mat::eye(3,3,CV_32FC1);
		transMat=Mat::zeros(3,1,CV_32FC1);			
		rs_obj.rotMat=rotMat;
		rs_obj.transMat=transMat;
		rs_obj.q=r2q(rotMat);
		rs_obj.SolutionState=state;
		rs_obj.status=4; //continue with all angles and translation as 0
		return rs_obj;
	}
	//LOGD<<"Marker 6.1";
	Mat pset1_mat; //3xn matrix
	for(unsigned int i=0;i<op_pset1.size();i++)
	{
		Mat temp31=(Mat_<float>(3,1) << op_pset1.at(i).x, op_pset1.at(i).y, op_pset1.at(i).z); 
		if (pset1_mat.rows == 0)
			temp31.copyTo(pset1_mat);
		else
			cv::hconcat(pset1_mat,temp31,pset1_mat);
	}
	
	Mat pset2_mat; //3xn matrix
	for(unsigned int i=0;i<op_pset2.size();i++)
	{
		Mat temp31=(Mat_<float>(3,1) << op_pset2.at(i).x, op_pset2.at(i).y, op_pset2.at(i).z);
		if (pset2_mat.rows == 0)
			temp31.copyTo(pset2_mat);
		else
			cv::hconcat(pset2_mat,temp31,pset2_mat);
	}
	
	Mat trans_rep;
	for(unsigned int i=0;i<op_pset2.size();i++)
	{
		if (trans_rep.rows == 0)
			transMat.copyTo(trans_rep);
		else
			cv::hconcat(trans_rep,transMat,trans_rep);
	}

	//LOGD<<"Marker 7";
	Mat ErrorRANSAC=rotMat*pset2_mat + trans_rep - pset1_mat; //3xn = 3x3*3xn + 3xn - 3xn  
	Mat ER_sq1=matperElem(ErrorRANSAC.row(0), ErrorRANSAC.row(0), "mul");
	Mat ER_sq2=matperElem(ErrorRANSAC.row(1), ErrorRANSAC.row(1), "mul");
	Mat ER_sq3=matperElem(ErrorRANSAC.row(2), ErrorRANSAC.row(2), "mul");
	
	Mat dummy=Mat::zeros(1,1,CV_32FC1);
	Mat ErrorRANSACNorm=matperElem(ER_sq1+ER_sq2+ER_sq3,dummy,"sqrt");//1xn //supply a dummy argument since its a uniary operation
	//LOGD<<"Marker 8";
	//float Error_mean=0;
	//for (int i=0;i<ErrorRANSACNorm.cols;i++)
	//{
	//	Error_mean=Error_mean+ErrorRANSACNorm.at<float>(1,i);
	//}
	//Error_mean=Error_mean/ErrorRANSACNorm.cols;

	cv::Scalar Errormean,Errorstd; //0:1st channel, 1:2nd channel and 2:3rd channel
	
	meanStdDev(ErrorRANSACNorm,Errormean,Errorstd,cv::Mat());
	rs_obj.nIterationRansac = min(rst,nIterations);
	rs_obj.nSupport = op_pset1.size();
	rs_obj.ErrorMean = Errormean.val[0];
	rs_obj.ErrorStd= Errorstd.val[0];
	rs_obj.SolutionState=state;
	rs_obj.status=1;
	step_n_1.mat_img.copyTo(rs_obj.mat_img1);
	step_n.mat_img.copyTo(rs_obj.mat_img2);


	//LOGD<<"Marker 9";
	if (rs_obj.nMatches != 0)
	{
		rs_obj.InlierRatio = ((float)rs_obj.nSupport/(float)rs_obj.nMatches)*100.0;
	}
	
	rs_obj.rotMat=rotMat;
	rs_obj.transMat=transMat;
	rs_obj.q=r2q(rotMat);

	//LOGD<<"rs_obj.nF1_Confidence_Filtered: "<<rs_obj.nF1_Confidence_Filtered;
	//LOGD<<"rs_obj.GoodFrames1.size: "<<rs_obj.GoodFrames1.size();
	if (state > 0)
	{
		rs_obj.SolutionState=state;
		rs_obj.status=1;
		return rs_obj;
	}
	else if (state < 1)
	{
		rotMat=Mat::eye(3,3,CV_32FC1);			
		transMat=Mat::zeros(3,1,CV_32FC1);
		rs_obj.rotMat=rotMat;
		rs_obj.transMat=transMat;
		rs_obj.q=r2q(rotMat);

		rs_obj.SolutionState=state;

		if (rs_obj.status != 4)
		{
			rs_obj.status=-1;
		}
		cout<<"vodometery : RANSAC failed.\n";
		return rs_obj;
	}
	//LOGD<<"Marker 10";
	return rs_obj;
}
//********************************************************************************************************
ransac_stat publish_sr4000(sr4000handler *sh, Engine *ep)
{
	bool fp_exists_last=true, fp_exists_curr=true;		
	ransac_stat rs_obj;	
	sr4000data data_obj_last;
	//cout<<"\nvo_publisher.cpp: marker 1.1\n";
	sh->getXYZObs(data_obj_last, fp_exists_last); //this function increments the file index internally
	//cout<<"vo_publisher.cpp:sh->queue_index_xyz: "<<sh->queue_index_xyz;
	//cout<<"\nvo_publisher.cpp: marker 1.2\n";
	if (fp_exists_last)
	{
		//cv::Mat den_image=p.sh->denoiseImage(data_obj.mat_img);
		cv::Mat den_image=sh->normalizeImage(data_obj_last.mat_img); //denoise and normalize image since it is almost always required in this form
		data_obj_last.den_norm_img=sh->gblurImage(den_image);
		//putText(data_obj.den_norm_img,str_idx, cvPoint(40,40),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
		sh->data_queue.push_back(data_obj_last);
	}
	//cout<<"\nvo_publisher.cpp: marker 1.3\n";
	sr4000data data_obj_curr;

	sh->getXYZObs(data_obj_curr, fp_exists_curr); //this function increments the file index internally
	cout<<"vo_publisher.cpp:sh->queue_index_xyz: "<<sh->queue_index_xyz;
	//cout<<"\nvo_publisher.cpp: marker 1.4\n";
	if (fp_exists_curr)
	{
		//cv::Mat den_image=p.sh->denoiseImage(data_obj.mat_img);
		cv::Mat den_image=sh->normalizeImage(data_obj_curr.mat_img); //denoise and normalize image since it is almost always required in this form
		data_obj_curr.den_norm_img=sh->gblurImage(den_image);
		//putText(data_obj.den_norm_img,str_idx, cvPoint(40,40),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
		sh->data_queue.push_back(data_obj_curr);
	}

	//cout<<"\nvo_publisher.cpp: marker 1.5\n";
	int state=1;
	if ((fp_exists_last == true) && (fp_exists_curr == true))
	{
		//rs_obj=vodometery(data_obj_last, data_obj_curr, state, sh->obs_no);
		rs_obj=vodometery(sh->obs_no,state,ep);
	}

	//cout<<"\nvo_publisher.cpp: marker 1.6\n";
	sh->obs_no++;
	return rs_obj;
}
//********************************************************************************************************
int main(int argc, char** argv)
{
	ros::init(argc, argv, "vo_publisher");

	ros::NodeHandle n;
	ros::Publisher vo_pub=n.advertise<nav_msgs::Odometry>("/vo", 60);
        ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 50);
	//ros::Publisher Wodom_pub = n.advertise<nav_msgs::Odometry>("odom", 60);	
	tf::TransformBroadcaster Iodom_broadcaster;
	tf::TransformBroadcaster odom_broadcaster;

	int count = 0;
	ros::Rate r(2.0);

	sr4000handler *sh= new sr4000handler("/home/emaad22/Datasets/SR4k_data");
	//cout<<"vo_publisher.cpp:sh->queue_index_xyz: "<<sh->queue_index_xyz;
	ransac_stat rs_obj;
	int limit_obs=20;

	ros::Time current_time1, current_time2, last_time1, last_time2;
	//************************** SETUP IMU *****************************
	VN_ERROR_CODE errorCode;
	Vn100 vn100;

	errorCode = vn100_connect(
        &vn100,
        COM_PORT,
        BAUD_RATE);
	
	/* Make sure the user has permission to use the COM port. */
	if (errorCode == VNERR_PERMISSION_DENIED) {
		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running again using 'sudo'.\n");

		return 0;
	}
	else if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to connect to the sensor.\n");

		return 0;
	}
	
	/* Configure the VN-100 to output asynchronous data. */
	errorCode = vn100_setAsynchronousDataOutputType(
        &vn100,
        VNASYNC_VNYPR,
        true);

	/* Pause to ensure we have received the first asynchronous data record
	   from the sensor. */
	//sleep(1);
	//********************************END IMU SETUP**********************************
	while ((count < limit_obs) && (n.ok()))
	{
		ros::spinOnce();		// check for incoming messages
		current_time1 = ros::Time::now();

		//cout<<"\nvo_publisher.cpp: marker 1\n";
		rs_obj=publish_sr4000(sh,ep);
		//cout<<"\nvo_publisher.cpp: marker 2\n";

		++count;
		r.sleep();
		//**************************************************
		current_time2 = ros::Time::now();
		current_time1 = ros::Time::now();		
		//***********************START TF TRANSFORM FOR IMU*********************
		//first, we'll publish the Wodom transform over tf
		tf::StampedTransform transform;

		//Transformation necessary to the robot_pose_ekf node
		transform.stamp_=current_time1;
		transform.frame_id_="base_footprint";
		transform.child_frame_id_ = "/imu_frame";        
		tf::Vector3 transl(0,0,0);
		transl[0]=0; 
		transl[1]=0; 
		transl[2]=0;
		transform.setOrigin(transl);

		tf::Quaternion odom_quat_ = tf::createQuaternionFromRPY(0,0,0);
		transform.setRotation(odom_quat_);
		//Publish tf
		Iodom_broadcaster.sendTransform(transform);
		//***********************END TF TRANSFORM FOR IMU********************
		//********************** EXTRACT INFO FROM IMU***********************
		VnDeviceCompositeData data;
		/* The library is handling and storing asynchronous data by itself.
		   Calling this function retrieves the most recently processed
		   asynchronous data packet. */
		vn100_getCurrentAsyncData(
			&vn100,
			&data);

		printf("  \n%+#7.2f %+#7.2f %+#7.2f\n",
			data.ypr.yaw,
			data.ypr.pitch,
			data.ypr.roll);

		//sleep(1); Give some time to the sensor to be able to listen again
		//***********************END INFO EXTRACTION***************************
   		//next, we'll publish the IMU message over ROS
		//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(yaw,-pitch,-roll);
		geometry_msgs::Quaternion Iodom_quat = tf::createQuaternionMsgFromRollPitchYaw(data.ypr.roll,data.ypr.pitch,data.ypr.yaw);

    		sensor_msgs::Imu Iodom;
    		Iodom.header.stamp = current_time1;
		Iodom.header.frame_id = "/imu_frame";	
		Iodom.orientation=Iodom_quat;
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

		//***********************START TF TRANSFORM FOR VO*********************
		current_time2 = ros::Time::now();		
		
		//first, we'll publish the transform over tf
 		tf::StampedTransform transform_vo;

		//Transformation necessary to the robot_pose_ekf node
		transform_vo.stamp_=current_time2;
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
    		//nav_msgs::Odometry odom;
		nav_msgs::Odometry odom;
    		odom.header.stamp = current_time2;
		odom.header.frame_id = "/odom";		
    		//odom.child_frame_id = "base_link";
    		//odom.child_frame_id = "base_footprint";

		//set the position
		odom.pose.pose.position.x = rs_obj.transMat.at<float>(0,0);
		odom.pose.pose.position.y = rs_obj.transMat.at<float>(1,0);
		odom.pose.pose.position.z = rs_obj.transMat.at<float>(2,0);
		geometry_msgs::Quaternion odom_quat;
		tf::Quaternion tfq=tf::Quaternion(rs_obj.q.at<float>(1,0), rs_obj.q.at<float>(2,0), rs_obj.q.at<float>(3,0), rs_obj.q.at<float>(0,0));		
		quaternionTFToMsg (tfq, odom_quat);
		odom.pose.pose.orientation = odom_quat;

		//compute odometry in a typical way given the velocities of the robot
		//double dt = (current_time2 - last_time2).toSec();
		//set the velocity
		//odom.twist.twist.linear.x = 0.1; //TBD: calculated via dist/time or s=vt formula
		//odom.twist.twist.linear.y = 0.1;
		//odom.twist.twist.angular.z = 1.0;

		
		// covariance
		//SymmetricMatrix covar =  filter_->PostGet()->CovarianceGet();
		//for (unsigned int i=0; i<6; i++)
		//	for (unsigned int j=0; j<6; j++)
		//		odom.twist.covariance[6*i+j] = pow(1.7,2);

		// covariance
		for (unsigned int i=0; i<6; i++)
			for (unsigned int j=0; j<6; j++)
				if (i == j)
					odom.pose.covariance[6*i+j] = pow(0.00017,2);
	
		//publish the message
		vo_pub.publish(odom);
		last_time1 = current_time1;
		last_time2 = current_time2;
		/////////////////////////////////////////
		++count;
		r.sleep();
	}

	errorCode = vn100_disconnect(&vn100);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}
}
