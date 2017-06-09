/*

 PROJECT				:	"Mono-SLAM EKF"
 MODULE					:	"ransac_stat"
 DEVELOPED BY			:	Muhammad Emaduddin
 CREATION DATE			:	21-01-2016

 MODIFIED BY			:	
 MODIFIED DATE			:	
 METHODS				:
 LINE OF CODE			:	----- LOC


 Copyright (c) 2016 EPSCOR-UALR (TBD).

 */
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
# pragma once

//*********************************************************************************
//System Header Include

//*********************************************************************************
//Class Decleration
class ransac_stat
{
public:
	ransac_stat(){
	nFeatures1=0;
	nF1_Confidence_Filtered=0;
	nFeatures2=0;
	nF2_Confidence_Filtered=0;
	nMatches=0;
	nIterationRansac=0;
	InlierRatio=0.0;
	nSupport=0;
	ErrorMean=0.0;
	ErrorStd=0.0;
	SolutionState=0;
	rotMat=cv::Mat::eye(3,3,CV_32FC1);			
	transMat=cv::Mat::zeros(3,1,CV_32FC1);
	q=cv::Mat::zeros(4,1,CV_32FC1);
	q.at<float>(0,0)=1;
	status=99;
	}

	int nFeatures1;
	int nF1_Confidence_Filtered;
	int nFeatures2;
	int nF2_Confidence_Filtered;
	int nMatches;
	int nIterationRansac;
	float InlierRatio;
	int nSupport;
	float ErrorMean;
	float ErrorStd;
	int SolutionState;

	std::vector<cv::KeyPoint> GoodFrames1; //x-range bw 0-176
	std::vector<cv::KeyPoint> GoodFrames2; //y-range bw 0-143
	std::vector<cv::KeyPoint> match;
	cv::Mat GoodDescp1;
	cv::Mat GoodDescp2;

	std::vector<cv::Point3f> op_pset1;
	cv::Mat pset1;
	std::vector<cv::Point3f> op_pset2;
	cv::Mat pset2;

	cv::Mat mat_img1;
	cv::Mat mat_img2;

	int status;
	cv::Mat rotMat; //The matrix holds rotation matrix estimate for current iteration populated via function featurehandler::vodometry. Its 3x3 matrix.
	cv::Mat transMat; //The matrix holds translation matrix estimate for current iteration populated via function featurehandler::vodometry. Its a 3x1 matrix.
	cv::Mat q; //The matrix holds rotation in quaternion representation. Its a 4x1 matrix.
};
//**********************************************************************************
