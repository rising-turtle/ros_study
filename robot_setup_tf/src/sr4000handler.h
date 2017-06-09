/*

 PROJECT				:	"Mono-SLAM EKF"
 MODULE					:	"sr4000handler"
 DEVELOPED BY			:	Muhammad Emaduddin
 CREATION DATE			:	10-01-2016

 MODIFIED BY			:	
 MODIFIED DATE			:	
 METHODS				:
 LINE OF CODE			:	----- LOC


 Copyright (c) 2016 EPSCOR-UALR (TBD).

 */
#include "sr4000data.h"
#include <opencv2/imgproc/imgproc.hpp>

//*********************************************************************************
//System Header Include

//*********************************************************************************

//Class Decleration
class sr4000handler
{
public:
	sr4000handler(std::string obs_path_input);

	size_t getDataQueueSize();
	void getObs(bool &fp_exists, cv::Mat &img, long int &timestamp);
	void getXYZObs(sr4000data &sd, bool &fp_exists);
	cv::Mat normalizeImage(cv::Mat input);
	cv::Mat denoiseImage(cv::Mat input);
	cv::Mat gblurImage(cv::Mat input);
	void start_capture_dataDS();

	std::vector<sr4000data> data_queue;
	
	std::string obs_path;
	int queue_index_xyz;
	bool initialized_xyz;
	int obs_no;

private:


};
//**********************************************************************************
