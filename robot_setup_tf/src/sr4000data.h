/*

 PROJECT				:	"Mono-SLAM EKF"
 MODULE					:	"sr4000data"
 DEVELOPED BY			:	Muhammad Emaduddin
 CREATION DATE			:	10-01-2016

 MODIFIED BY			:	
 MODIFIED DATE			:	
 METHODS				:
 LINE OF CODE			:	----- LOC

 Copyright (c) 2016 EPSCOR-UALR (TBD).
 */
#include <string>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

//*********************************************************************************
//System Header Include

//*********************************************************************************
//Class Decleration
class sr4000data
{
public:
	sr4000data()
	{}
	cv::Mat z; //144 x 176 , CV_32F
	cv::Mat x; //144 x 176 , CV_32F
	cv::Mat y; //144 x 176 , CV_32F
	cv::Mat mat_img;
	cv::Mat den_norm_img; //144 x 176 , CV_8U
	cv::Mat confidence_map;
	long int timestamp;
	int serial;

private:
	
};

//**********************************************************************************
