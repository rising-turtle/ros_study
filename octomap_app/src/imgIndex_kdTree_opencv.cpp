#include "imgIndex_kdTree_opencv.h"
//#include <Windows.h>

ImgIndexKdTreeOpenCV::ImgIndexKdTreeOpenCV(void)
{
}

ImgIndexKdTreeOpenCV::~ImgIndexKdTreeOpenCV(void)
{

}
void ImgIndexKdTreeOpenCV::addFeature(cv::Mat desc, int img_index)
{

}

void ImgIndexKdTreeOpenCV::addFeature()
{
//	m_matcher_SURF.clear();


	m_matcher_SURF.add(m_feat_pool_SURF);


}
void ImgIndexKdTreeOpenCV::build()
{
	
	m_matcher_SURF.train();

}
void ImgIndexKdTreeOpenCV::quiryLoc(cv::Mat desc, OUTPUT_PARAM& dst)
{
//	dst.bOK = true;
	if (m_feat_size != desc.cols)
	{
//		dst.bOK = true;
		dst.loc_x[0] = 99;
		dst.loc_y[0] = m_feat_size;
		dst.loc_z[0] = desc.cols;
		return;
	}
	if(  m_matcher_SURF.empty())
	{
		dst.loc_y[0] = 99;
		return;
	}
	if (desc.type()!=CV_32F)
	{
		desc.convertTo(desc, CV_32F);
	}
	
	memset(&dst, 0, sizeof(OUTPUT_PARAM));
	dst.loc_x[0] = 0;
	dst.loc_y[0] = 0;
	dst.loc_z[0] = 0;

	vector<cv::DMatch> matches;
	m_matcher_SURF.match(desc, matches);
	voteBestLoc(matches, dst);
}

void ImgIndexKdTreeOpenCV::quiryLoc2D(cv::Mat desc, OUTPUT_PARAM& dst)
{
	//LARGE_INTEGER freq;  
	//LARGE_INTEGER start_t, stop_t;  
	//double exe_time;
	//QueryPerformanceFrequency(&freq);  
	//QueryPerformanceCounter(&start_t);  
	if (desc.rows==0)
	{
		dst.bOK = false;
		return;
	}
	if (m_feat_size != desc.cols)
	{
		//		dst.bOK = true;
		dst.loc_x[0] = 99;
		dst.loc_y[0] = m_feat_size;
		dst.loc_z[0] = desc.cols;
		return;
	}
	if(  m_matcher_SURF.empty())
	{
		dst.loc_y[0] = 99;
		return;
	}
	if (desc.type()!=CV_32F)
	{
		desc.convertTo(desc, CV_32F);
	}

	memset(&dst, 0, sizeof(OUTPUT_PARAM));
	dst.loc_x[0] = 0;
	dst.loc_y[0] = 0;
	dst.loc_z[0] = 0;

	
	vector<cv::DMatch> matches;

	//QueryPerformanceCounter(&stop_t);  
	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage6-1 cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[6][1][0] += exe_time;
	//m_time_cnt[6][1][0]++;

	//QueryPerformanceCounter(&start_t);  
	m_matcher_SURF.match(desc, matches);

	//QueryPerformanceCounter(&stop_t);  
	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage6-2-feat match cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[6][2][0] += exe_time;
	//m_time_cnt[6][2][0]++;
	//QueryPerformanceCounter(&start_t);  
	voteBestLoc2D(matches, dst);
	//QueryPerformanceCounter(&stop_t);  
	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage6-3-voteBestLoc cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[6][3][0] += exe_time;
	//m_time_cnt[6][3][0]++;
}

void ImgIndexKdTreeOpenCV::voteBestLoc2D(vector<cv::DMatch> matches, OUTPUT_PARAM& dst)
{
	//LARGE_INTEGER freq;  
	//LARGE_INTEGER start_t, stop_t;  
	//double exe_time;
	//QueryPerformanceFrequency(&freq);  
	//QueryPerformanceCounter(&start_t);  

	sort(matches.begin(), matches.end());
	vector<cv::Point3f> object_points;
	vector<cv::Point2f> image_points;
	int match_num = (int)matches.size();
	cv::Point3f p_3d;
	int idx_img = 0;
	for (int i=0; i<match_num; i++)
	{
		if (matches[i].distance>0.2)
		{
			break;
		}	
		idx_img = matches[i].imgIdx;
		p_3d.x = m_map_feat_locs[idx_img](0);
		p_3d.y = m_map_feat_locs[idx_img](1);
		p_3d.z = m_map_feat_locs[idx_img](2);
		object_points.push_back(p_3d);
		image_points.push_back(m_quiry_locs_2d[matches[i].queryIdx]);
	}
	cout<<"matched num:                 "<<image_points.size()<<endl;
	
	cv::Mat rvec;// = cv::Mat::zeros(3, 1, CV_32FC1);//x,y,z rotation: roll, pitch, yaw
	cv::Mat tvec;// = cv::Mat::zeros(3, 1, CV_32FC1);
	//bool use_extrinsic_guess = false;
	//int iterations_count = 100;
	//float reprojection_error = 5.0;
	//int min_inliers_count = 5;
	//cv::Mat inliner;
	double conf = 0;
	//QueryPerformanceCounter(&stop_t);  

	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage6-3-1 cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[6][3][1] += exe_time;
	//m_time_cnt[6][3][1]++;

	//QueryPerformanceCounter(&start_t);  
	bool flag = getTransformationRANSAC2D(object_points, image_points, rvec, tvec, conf);
	//QueryPerformanceCounter(&stop_t);  
	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage6-3-2 ransac cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[6][3][2] += exe_time;
	//m_time_cnt[6][3][2]++;
	//QueryPerformanceCounter(&start_t);  
	if (!flag)
	{

		dst.bOK = false;
		return;
	}
	else dst.bOK = true;
	cv::Mat matrix_rotation;
	cv::Rodrigues(rvec, matrix_rotation);
	cv::invert(matrix_rotation, matrix_rotation);
	cv::Mat _tvec = matrix_rotation * tvec;
//	tvec = _tvec;
	//dst.loc_x[0] = -_tvec.at<double>(0,0); 
	//dst.loc_y[0] = -_tvec.at<double>(2,0);
	//dst.loc_z[0] = _tvec.at<double>(1,0);
	dst.loc_x[0] = -_tvec.at<double>(0,0); // new direction
	dst.loc_y[0] = -_tvec.at<double>(1,0);
	dst.loc_z[0] = -_tvec.at<double>(2,0);
	dst.angle_roll[0] = rvec.at<double>(0,0);
	dst.angle_pitch[0] = rvec.at<double>(1,0);
	dst.angle_yaw[0] = rvec.at<double>(2,0);
	dst.confidence[0] = conf;

	//QueryPerformanceCounter(&stop_t);  
	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage6-3-3 final cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[6][3][3] += exe_time;
	//m_time_cnt[6][3][3]++;
}

bool ImgIndexKdTreeOpenCV::getTransformationRANSAC2D(vector<cv::Point3f> object_points, vector<cv::Point2f> image_points, cv::Mat& rvec, cv::Mat& tvec, double& mean_err)
{
	bool flag = false;
	int ransac_iterations = 80;
	const unsigned int sample_size = 4;
	double inlier_error = 0;
	const float max_dist_m = 10;//2;//��������ƫ��;
	unsigned int min_inlier_thrd = 8;//��С���ڵ�����ֵ;
	int match_size = object_points.size();
	if (match_size != image_points.size())
	{
		return flag;
	}
	if (min_inlier_thrd>0.75*match_size)
	{
		min_inlier_thrd = (unsigned int)0.75 * match_size;
		return flag;
	}
	std::vector<int> matches_final;
	double rmse = 1e6;
	double refined_error = 1e6;
	cv::Mat refined_rvec;
	cv::Mat refined_tvec;
	int size_thrd = 0;
	int refined_mt_size = 0;
	for (int n=0; (n<ransac_iterations && match_size>=sample_size); n++)
	{
		refined_error = 1e6;
		std::vector<int> refined_matches;
		std::vector<int> inlier_index = sample_matches_2D(sample_size, match_size);//initialization with random samples
		if (inlier_index.size()<sample_size)
		{
			break;
		}

		for(int refinements = 0; refinements < 5 /*got stuck?*/; refinements++)
		{
			cv::Mat trans_R, trans_T;
			if (!getTransformFromMatchesSolvePnP(object_points, image_points, inlier_index, trans_R, trans_T))
			{
				continue;
			}
			//test which features are inliers
			computeInliersAndError2D(object_points, image_points, trans_R, trans_T, 
				inlier_index, inlier_error, max_dist_m*max_dist_m/(1+refinements/1.0));	

			if(inlier_index.size() < min_inlier_thrd || inlier_error > max_dist_m)
			{
				break; //hopeless case
			}

			//superior to before?
			size_thrd = refined_matches.size() * 0.8;
			size_thrd = (size_thrd>min_inlier_thrd) ? size_thrd : min_inlier_thrd;
			if (inlier_index.size() >= size_thrd && inlier_error < refined_error)
			{
				refined_rvec = trans_R;
				refined_tvec = trans_T;
				refined_matches = inlier_index;
				refined_error = inlier_error;
			}
			else break;
		}  //END REFINEMENTS
		//Successful Iteration?
		refined_mt_size = refined_matches.size();
		if(refined_mt_size >= min_inlier_thrd)  //Valid?
		{

			//Acceptable && superior to previous iterations?
			if (refined_error < rmse &&
				refined_mt_size > matches_final.size() &&
				refined_mt_size >= min_inlier_thrd)
			{
				flag = true;
				rmse = refined_error;
				rvec = refined_rvec;
				tvec = refined_tvec;
				matches_final = refined_matches;

				//Performance hacks:
				double percentage_of_inliers = refined_matches.size() * 100.0 / object_points.size();
				if (percentage_of_inliers > 60.0) break; ///Can this get better anyhow?

			}
		}

	}
	int inlier_num = matches_final.size();
	if(rmse>8 || inlier_num<min_inlier_thrd || (rmse>2 && inlier_num<min_inlier_thrd*1.5))
	{
		flag = false;
	}
	double rat = inlier_num / 30.0;
	if (rat>3)
	{
		rat = 3;
	}
	mean_err = rat * (8 - rmse) / 8;
	mean_err = (mean_err>1) ? 1 : mean_err;
	cout<<"mean errors: "<<rmse<<'\t'<<"final matched num: "<<matches_final.size()<<'\n';
	return flag;
}

bool ImgIndexKdTreeOpenCV::getTransformFromMatchesSolvePnP(vector<cv::Point3f> object_points, vector<cv::Point2f> image_points, vector<int> match_index, cv::Mat& trans_R, cv::Mat& trans_T)
{

	vector<cv::Point3f> p_obj;
	vector<cv::Point2f> p_img;
	int size_inlier = match_index.size();
	if (size_inlier<4)
	{
		return false;
	}
	for (int i=0; i<size_inlier; i++)
	{
		p_obj.push_back(object_points[match_index[i]]);
		p_img.push_back(image_points[match_index[i]]);
	}
	bool flag = cv::solvePnP(p_obj, p_img, m_camera_mat, m_dist_coef, trans_R, trans_T);
	return flag;
}
void ImgIndexKdTreeOpenCV::voteBestLoc(vector<cv::DMatch> matches, OUTPUT_PARAM& dst)
{
	cout<<"matched num:            "<<matches.size()<<endl;
	vector<cv::DMatch> matches_with_depth;
	vector<cv::DMatch>::iterator iter=matches.begin();
	
	while(iter!=matches.end())
	{

		float d1 = m_quiry_locs[iter->queryIdx](2);
		float d2 = m_map_feat_locs[iter->imgIdx](2);
		if (d1>0.5 && d1<7 && d2 && (iter->distance<0.2))//�������ƶ���ֵ0.6��Ҫ����;
		{
			matches_with_depth.push_back(*iter);
		}		
//		cout<<"feature loc:"<<m_quiry_locs[iter->queryIdx](0)<<'\t'<<m_quiry_locs[iter->queryIdx](1)<<'\t'<<m_quiry_locs[iter->queryIdx](2)<<'\n';
/*		if(!(d1>0.01) || !(d1<5))
			cout<<"quiry_depth:"<<d1<<'\t';
		if(!d2)
			cout<<"map depth"<<d2<<'\t';
		if(!(iter->distance<1))
			cout<<"distance"<<'\t';*/

		iter++;
	}
//	cout<<endl;

	cout<<"matched feature with depth num:"<<matches_with_depth.size()<<'\n';
 	sort(matches_with_depth.begin(), matches_with_depth.end());
//	sort(matches.begin(), matches.end());
//	if(matches.size()>0)
//		cout<<"nearest distance:::::::::::::::::::::::::::"<<matches[0].distance<<endl;
//	if(matches_with_depth.size()>0)
//		cout<<"nearest distance with depth:::::::::::::::::::"<<matches_with_depth[0].distance<<endl;
	//////////////////////////////////////////////////////////////////////////
	//estimate the transformation between the current image and submap
	Eigen::Matrix4f trans;// = Eigen::Matrix4f::Identity();
//	bool flag = true;
	dst.bOK = true;
	float mean_err = 1000000;
	trans = getTransformationRANSAC(m_quiry_locs, m_map_feat_locs, matches_with_depth, mean_err, dst.bOK);

	if(!dst.bOK)
	{
		dst.loc_x[0] = 0;
		dst.loc_y[0] = 0;
		dst.loc_z[0] = 0;
//		cout<<"quiry flag = false"<<endl;
		dst.confidence[0] = 0;
		return;
	}

	Eigen::Vector4f origin_pos(0,0,0,1);
	Eigen::Vector3f dst_pos = (trans * origin_pos).head<3>();
	dst.loc_x[0] = dst_pos(0);
	dst.loc_y[0] = dst_pos(1);
	dst.loc_z[0] = dst_pos(2);
	dst.confidence[0] = (3 - mean_err) / 3;
	double roll, pitch, yaw;
	matrix2RPY(trans, roll, pitch, yaw);
	dst.angle_roll[0] = roll + PI / 2;
	dst.angle_pitch[0] = pitch;
	dst.angle_yaw[0] = yaw;
	if(dst.confidence[0]<0)
		dst.confidence[0] = 0;
	else if(dst.confidence[0]>1)
		dst.confidence[0] = 1;
}

Eigen::Matrix4f ImgIndexKdTreeOpenCV::getTransformationRANSAC(std_vector_of_eigen_vector4f locs_quiry, std_vector_of_eigen_vector4f locs_map, vector<cv::DMatch> matches, float& mean_err, bool& flag)
{
	flag = false;
//	Eigen::Matrix4f trans;
	int ransac_iterations = 10;
	const unsigned int sample_size = 3;
	double inlier_error = 0;
	const float max_dist_m = 1.5;//2;//��������ƫ��;
	unsigned int min_inlier_thrd = 10;//��С���ڵ�����ֵ;
	Eigen::Matrix4f resulting_transformation = Eigen::Matrix4f::Zero(4,4);//  = Eigen::Matrix4f::Identity();;
	
	if (min_inlier_thrd>0.75*matches.size())
	{
		min_inlier_thrd = (unsigned int)0.75 * matches.size();
	}
	std::vector<cv::DMatch> matches_final;
	double rmse = 1e6;
//	cout<<"begin ransac  "<< matches.size()<<endl;
	for (int n=0; (n<ransac_iterations && matches.size()>=sample_size); n++)
	{
//		cout<<"ransac iteration   "<< n<<'\t'<< n<<'\t'<< n<<'\t'<< n<<'\t'<< n<<'\t'<< n<<'\t'<< n<<endl;
		double refined_error = 1e6;
		std::vector<cv::DMatch> refined_matches;
		std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, matches); //initialization with random samples
		//std::vector<cv::DMatch> inlier = sample_matches(sample_size, matches_with_depth); //initialization with random samples
		Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();
//		cout<<"initial inlier num: "<<inlier.size()<<endl;
	//	real_iterations++;
		for(int refinements = 1; refinements < 5 /*got stuck?*/; refinements++)
		{
//			cout<<"refinements round  "<<refinements<<endl;
			Eigen::Matrix4f transformation = getTransformFromMatchesUmeyama(locs_quiry, locs_map, inlier);

			//test which features are inliers
			computeInliersAndError(matches, transformation,
				locs_quiry, //this->feature_depth_stats_,
				locs_map, //earlier_node->feature_depth_stats_,
				inlier, inlier_error, max_dist_m*max_dist_m/(1+refinements/1.0));//���ŵ��������ӣ���ֵ�𽥼�С����߱�׼��;
//			cout<<"inlier_error: "<< inlier_error<<" inlier size: "<<inlier.size()<<endl;

			if(inlier.size() < min_inlier_thrd || inlier_error > max_dist_m)
			{
			//	ROS_DEBUG_NAMED(__FILE__, "Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
				break; //hopeless case
			}

			//superior to before?
			int size_thrd = refined_matches.size() * 0.8;
			size_thrd = (size_thrd>min_inlier_thrd) ? size_thrd : 10;
			if (inlier.size() > size_thrd && inlier_error < refined_error)
			{
				assert(inlier_error>=0);
				refined_transformation = transformation;
				refined_matches = inlier;
				refined_error = inlier_error;
			}
			else break;
		}  //END REFINEMENTS
		//Successful Iteration?
		if(refined_matches.size() > 0)  //Valid?
		{
//			cout<<"0000000000000000000000000000 get good transformation"<<"refined matches size:   "<<refined_matches.size()<<endl;
	//		valid_iterations++;
	//		ROS_DEBUG("Valid iteration: inliers/matches: %lu/%lu (min %u), refined error: %.2f (max %.2f), global error: %.2f",
	//			refined_matches.size(), matches.size(), min_inlier_threshold,  refined_error, max_dist_m, rmse);

			//Acceptable && superior to previous iterations?
			if (refined_error < rmse &&
				refined_matches.size() > matches_final.size() &&
				refined_matches.size() >= min_inlier_thrd)
			{
//				cout<<"get transformation "<<"matches size:"<<refined_matches.size()<<" refined error:"<<refined_error<<endl;
//				ROS_INFO("Improvment in iteration %d: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", real_iterations, (int)refined_matches.size(), (int) min_inlier_threshold,  refined_error, max_dist_m);
				flag = true;
				rmse = refined_error;
				resulting_transformation = refined_transformation;
				matches_final.assign(refined_matches.begin(), refined_matches.end());
				//Performance hacks:
				double percentage_of_inliers = refined_matches.size() * 100.0 / matches.size();
				if (percentage_of_inliers > 60.0) break; ///Can this get better anyhow?
				
			}
		}

	}
	if(rmse>0.4)
	{
		flag = false;
		resulting_transformation = Eigen::Matrix4f::Zero(4,4);
//		cout<<"eeeerrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"<<endl;
	}
	//if(flag)
	// cout<<"RANSAC flag= true"<<endl;
	//else
	//	cout<<"RANSAC flag= false"<<endl;
	mean_err = rmse;
	cout<<"mean errors: "<<rmse<<'\t'<<"final matched num: "<<matches_final.size()<<'\n';
//	cout<<;
//	trans = getTransformFromMatchesUmeyama(locs_quiry, locs_map, matches);
	return resulting_transformation;
}
Eigen::Matrix4f ImgIndexKdTreeOpenCV::getTransformFromMatchesUmeyama(std_vector_of_eigen_vector4f locs_quiry, std_vector_of_eigen_vector4f locs_map, vector<cv::DMatch> matches)
{
	Eigen::Matrix<float, 3, Eigen::Dynamic> tos(3, matches.size()), froms(3, matches.size());
	vector<cv::DMatch>::const_iterator it = matches.begin();
	for (int i=0; it!=matches.end(); it++, i++)
	{
		Eigen::Vector3f f = locs_quiry[it->queryIdx].head<3>();
		Eigen::Vector3f t = locs_map[it->imgIdx].head<3>();
		if (!(f(2)) || !(t(2)))
		{
			continue;
		}
		froms.col(i) = f;
		tos.col(i) = t;
	}
	Eigen::Matrix4f dst = Eigen::Matrix4f::Identity();
	dst = Eigen::umeyama(froms, tos, false);
	return dst;
}
void ImgIndexKdTreeOpenCV::initial(string path_global_map, int feat_size, float thrd)
{
    // ros init
//	int argc=0;
//	char **argv=NULL;
 //   ros::init(argc, argv, "main_arDetector");

//	m_algo_ar_server_ = new ARServer;
//	m_algo_ar_server_->arInit();

	m_detector_surf = new cv::SurfFeatureDetector(1000);
	m_descriptor_surf =  new cv::SurfDescriptorExtractor;

	m_feat_size = feat_size;
	
	m_thrd = thrd;
	m_data_ptr = 0;
	m_max_img_index = 0;
	m_matcher_SURF.clear();

	float fu = 515.737;
	float fv = 516.428;
	float cx = 320;//317.235;
	float cy = 240;//239.844;
	m_camera_mat = cv::Mat::eye(3, 3, CV_32FC1);
	//randu(camera_mat, 0.5, 1);
	m_camera_mat.at<float>(0, 1) = 0.f;
	m_camera_mat.at<float>(1, 0) = 0.f;
	m_camera_mat.at<float>(2, 0) = 0.f;
	m_camera_mat.at<float>(2, 1) = 0.f;
	m_camera_mat.at<float>(0, 0) = fu;
	m_camera_mat.at<float>(1, 1) = fv;
	m_camera_mat.at<float>(0, 2) = cx;
	m_camera_mat.at<float>(1, 2) = cy;
	m_dist_coef = cv::Mat::zeros(5, 1, CV_32FC1);
	//m_dist_coef.at<float>(0, 0) = 0.0231;
	//m_dist_coef.at<float>(1, 0) = 0.1064;
	//m_dist_coef.at<float>(2, 0) = 0.0033;
	//m_dist_coef.at<float>(3, 0) = 0.0061;

	readGlobalMap(path_global_map);
	addFeature();
	build();
	memset(m_time_cost, 0, 10*10*10*sizeof(double));
	memset(m_time_cnt, 0, 10*10*10*sizeof(int));
}
void ImgIndexKdTreeOpenCV::clean()
{
	m_data_ptr = 0;
	m_max_img_index = 0;
	delete m_detector_surf;
	delete m_descriptor_surf;
}
float ImgIndexKdTreeOpenCV::getDepthValue(cv::Mat img_depth, float x, float y, int winSize)
{
	int ix = (int)x;
	int iy = (int)y;
	unsigned short z = img_depth.at<unsigned short>(iy,ix);
	float dst_z = 0;
	int start_x, start_y, stop_x, stop_y, cnt;
	unsigned int sum_value;
	if (0==z)
	{
		start_y = max(0, iy - winSize);
		stop_y = min(img_depth.rows - 1, iy + winSize);
		start_x = max(0, ix - winSize);
		stop_x = min(img_depth.cols - 1, ix + winSize);
		sum_value = 0;
		cnt = 0;
		for (int m = start_y; m < stop_y; m++) {
			for (int n = start_x; n < stop_x; n++) {
				if (img_depth.at<unsigned short>(m, n) > 0) {
					sum_value += img_depth.at<unsigned short>(m, n);
					cnt++;
				}

			}
		}
		if (cnt > 0) {
			dst_z = sum_value / cnt;
		}
	}
	else
	{
		winSize = 3;
		start_y = max(0, iy - winSize);
		stop_y = min(img_depth.rows - 1, iy + winSize);
		start_x = max(0, ix - winSize);
		stop_x = min(img_depth.cols - 1, ix + winSize);
		sum_value = 0;
		cnt = 0;
		for (int m = start_y; m < stop_y; m++) {
			for (int n = start_x; n < stop_x; n++) {
				if (img_depth.at<unsigned short>(m, n) > 0) {
					sum_value += img_depth.at<unsigned short>(m, n);
					cnt++;
				}

			}
		}
		if (cnt > 0) {
			dst_z = sum_value / cnt;
		}
	}
	return dst_z;
}
void ImgIndexKdTreeOpenCV::getFeatLocation2D(vector<cv::KeyPoint> kp)
{
	m_quiry_locs_2d.clear();
	int pos_num = (int)kp.size();
	for (int i=0; i<pos_num; i++)
	{
		m_quiry_locs_2d.push_back(kp[i].pt);
	}
}
void ImgIndexKdTreeOpenCV::getFeatLocation(vector<cv::KeyPoint> kp, cv::Mat img_depth)
{
	m_quiry_locs.clear();
	for (int i=0; i<(int)kp.size(); i++)
	{			
		float x = kp[i].pt.x;
		float y = kp[i].pt.y;
		//unsigned short z = img_depth.at<unsigned short>(y,x);
		float z = getDepthValue(img_depth, x, y, 10);

		
		float fx = 0.00193897;//525.0;
		float fy = 0.00193638;//525.0;
		float cx = 317.235;//319.5;
		float cy = 239.844;//239.5;
		float factor = 1000; //for the 16-bit PNG files;
		float depth_scale = 1;
		Eigen::Vector4f fea_loc;
		fea_loc(2) = z / factor * depth_scale;
		fea_loc(0) = (x - cx) * fea_loc(2) * fx;
		fea_loc(1) = (y - cy) * fea_loc(2) * fy;
		fea_loc(3) = 1;
		m_quiry_locs.push_back(fea_loc);
	}
}
void ImgIndexKdTreeOpenCV::readSubmap(string file_name)
{
//	m_submap.clear();
//	resolve::SubmapHeader h;
//	ifstream inf(file_name.c_str());
//	inf>>h;
//	//	print(cout, h);
//	m_submap.read(file_name);
//	if (m_feat_size!=m_submap.m_feature_des.cols)
//	{
//		return;
//	}
//	if (m_submap.m_feature_des.type()!=CV_32F)
//	{
//		m_submap.m_feature_des.convertTo(m_submap.m_feature_des, CV_32F);
//	}
////	m_feat_pool_SURF.push_back(m_submap.m_feature_des);
//	Eigen::Quaternionf q(m_submap.m_rootPose.rw, m_submap.m_rootPose.rx, m_submap.m_rootPose.ry, m_submap.m_rootPose.rz);
//	Eigen::Matrix3f matrix_q3;// = q.toRotationMatrix();
//	Eigen::Matrix4f matrix_q4 = Eigen::Matrix4f::Identity();
//	matrix_q4(0,0) = matrix_q3(0,0);
//	matrix_q4(0,1) = matrix_q3(0,1);
//	matrix_q4(0,2) = matrix_q3(0,2);
//	matrix_q4(0,3) = m_submap.m_rootPose.tx;
//	matrix_q4(1,0) = matrix_q3(1,0);
//	matrix_q4(1,1) = matrix_q3(1,1);
//	matrix_q4(1,2) = matrix_q3(1,2);
//	matrix_q4(1,3) = m_submap.m_rootPose.ty;
//	matrix_q4(2,0) = matrix_q3(2,0);
//	matrix_q4(2,1) = matrix_q3(2,1);
//	matrix_q4(2,2) = matrix_q3(2,2);
//	matrix_q4(2,3) = m_submap.m_rootPose.tz;
//	matrix_q4(3,0) = 0;
//	matrix_q4(3,1) = 0;
//	matrix_q4(3,2) = 0;
//	matrix_q4(3,3) = 1;
//	for (int i=0; i<m_submap.m_feature_locs.size(); i++)
//	{
//		/*Eigen::Vector4f x_1;
//		x_1(0) = m_submap.m_feature_locs[i](0);
//		x_1(1) = m_submap.m_feature_locs[i](1);
//		x_1(2) = m_submap.m_feature_locs[i](2);
//		x_1(3) = 1;
//		Eigen::Vector4f global_pos = matrix_q4 * x_1;
//		m_map_feat_locs.push_back(global_pos);*/
//		m_submap.m_feature_locs[i](0) += m_submap.m_rootPose.tx;
//		m_submap.m_feature_locs[i](1) += m_submap.m_rootPose.ty;
//		m_submap.m_feature_locs[i](2) += m_submap.m_rootPose.tz;
//		m_submap.m_feature_locs[i](3) = 1;
//		m_map_feat_locs.push_back(m_submap.m_feature_locs[i]);
//
//		m_feat_pool_SURF.push_back(m_submap.m_feature_des.row(i));
//	}
	
}
void ImgIndexKdTreeOpenCV::readGlobalMap(string file_name)
{
	m_map_feat_locs.clear();
	m_feat_pool_SURF.clear();
	ifstream inf(file_name.c_str());
	int feat_num, feat_size;
	inf>>feat_num>>feat_size;

	printf("###############################feat_num: %d;  feat_size: %d\n", feat_num, feat_size);
	printf("###############################feat_num: %d;  feat_size: %d\n", feat_num, feat_size);
	if (m_feat_size!=feat_size)
	{
		return;
	}

	for(int i=0; i<feat_num; i++)
	{
		Eigen::Vector4f feat_loc;
		//inf>>feat_loc(0)>>feat_loc(1)>>feat_loc(2);
		inf>>feat_loc(0)>>feat_loc(1)>>feat_loc(2);//new coordination
		//feat_loc(2) =  - feat_loc(2);
		feat_loc(3) = 1;
		m_map_feat_locs.push_back(feat_loc);
		cv::Mat feat_des = cv::Mat(1, feat_size, CV_32FC1);
		for(int j=0;j<feat_size;j++)
		{
			inf>>feat_des.at<float>(0,j);
		}
		m_feat_pool_SURF.push_back(feat_des);
	}
}

///Randomly choose <sample_size> of the matches
vector<int> ImgIndexKdTreeOpenCV::sample_matches_2D(unsigned int sample_size, unsigned int total_size)
{
	//Sample ids to pick matches later on (because they are unique and the
	//DMatch operator< overload breaks uniqueness of the Matches if they have the
	//exact same distance, e.g., 0.0)
	int safety_net = 0;
	std::vector<int> sampled_index;
	while(sampled_index.size() < sample_size && total_size >= sample_size)
	{
		//generate a set of samples. Using a set solves the problem of drawing a sample more than once
		int id1 = rand() % total_size;
		int id2 = rand() % total_size;
		if(id1 > id2) id1 = id2; //use smaller one => increases chance for lower id
		sampled_index.push_back(id1);
	}
	return sampled_index;
}


///Randomly choose <sample_size> of the matches
vector<cv::DMatch> ImgIndexKdTreeOpenCV::sample_matches_prefer_by_distance(unsigned int sample_size, std::vector<cv::DMatch>& matches_with_depth)
{
	//Sample ids to pick matches lateron (because they are unique and the
	//DMatch operator< overload breaks uniqueness of the Matches if they have the
	//exact same distance, e.g., 0.0)
	std::set<std::vector<cv::DMatch>::size_type> sampled_ids;
	int safety_net = 0;
	std::vector<cv::DMatch> sampled_matches;
	while(sampled_ids.size() < sample_size && matches_with_depth.size() >= sample_size)
	{
		//generate a set of samples. Using a set solves the problem of drawing a sample more than once
		int id1 = rand() % matches_with_depth.size();
		int id2 = rand() % matches_with_depth.size();
		if(id1 > id2) id1 = id2; //use smaller one => increases chance for lower id
		sampled_ids.insert(id1);
		sampled_matches.push_back(matches_with_depth[id1]);
	}


	return sampled_matches;
}

//�򻯰棬ֱ�Ӽ���ŷʽ����;
double ImgIndexKdTreeOpenCV::errorsFunction2(const Eigen::Vector4f& x1,
					   const Eigen::Vector4f& x2,
					   const Eigen::Matrix4f& tf_1_to_2) const
{
	Eigen::Vector4d x_1 = x1.cast<double>();
	Eigen::Vector4d x_2 = x2.cast<double>();
	Eigen::Matrix4d tf_12 = tf_1_to_2.cast<double>();
	Eigen::Vector3d mu_1 = x_1.head<3>();
	Eigen::Vector3d mu_2 = x_2.head<3>();
	Eigen::Vector3d mu_1_in_2 = (tf_12 * x_1).head<3>();
	Eigen::Vector3d delta_mu_in_2 = mu_1_in_2 - mu_2;
	double dis = delta_mu_in_2(0)*delta_mu_in_2(0) + delta_mu_in_2(1)*delta_mu_in_2(1) + delta_mu_in_2(2)*delta_mu_in_2(2); 
	return dis;
}

void ImgIndexKdTreeOpenCV::computeInliersAndError(const std::vector<cv::DMatch> & all_matches,
								  const Eigen::Matrix4f& transformation,
								  std_vector_of_eigen_vector4f origins,
								  std_vector_of_eigen_vector4f earlier,
								  std::vector<cv::DMatch>& inliers, //pure output var
								  double& mean_error,//pure output var: rms-mahalanobis-distance
								  double squaredMaxInlierDistInM) const
{
	inliers.clear();
	//errors.clear();
	std::vector<std::pair<float,int> > dists;
	assert(all_matches.size() > 0);
	mean_error = 0.0;


	vector<cv::DMatch>::const_iterator iter = all_matches.begin();
	for(;iter!=all_matches.end(); iter++)
	{
		const Eigen::Vector4f& origin = origins[iter->queryIdx];
		const Eigen::Vector4f& target = earlier[iter->imgIdx];
		if(origin(2) == 0.0 || target(2) == 0.0 )
		{
			continue;
		}
		double mahal_dist = errorsFunction2(origin, target, transformation);
//		cout<<"detect loc:"<<origin(0)<<'\t'<<origin(1)<<'\t'<<origin(2)<<'\t'<<origin(3)<<endl;
//		cout<<"map loc:"<<target(0)<<'\t'<<target(1)<<'\t'<<target(2)<<'\t'<<target(3)<<endl;
//		cout<<"distance: "<<mahal_dist<<endl;
		if(mahal_dist > squaredMaxInlierDistInM)
			continue; //ignore outliers
		if(!(mahal_dist >= 0.0))
		{
			//ROS_WARN_STREAM("Mahalanobis_ML_Error: "<<mahal_dist);
			//ROS_WARN_STREAM("Transformation for error !>= 0:\n" << transformation << "Matches: " << all_matches.size());
			continue;
		}
		inliers.push_back(*iter); //include inlier
		mean_error += mahal_dist;
	}

	if (inliers.size()<3)  //at least the samples should be inliers
	{
		//ROS_WARN_COND(inliers.size() > 3, "No inliers at all in %d matches!", (int)all_matches.size()); // only warn if this checks for all initial matches
		mean_error = 1e9;
	}
	else
	{
		mean_error /= inliers.size();
		mean_error = sqrt(mean_error);
	}

}

void ImgIndexKdTreeOpenCV::computeInliersAndError2D(vector<cv::Point3f> object_points,
													vector<cv::Point2f> image_points, 
													cv::Mat rvec_src,
													cv::Mat tvec_src, 
													vector<int>& inliers_dst,
													double& mean_err_dst,
													double squaredMaxInlierDistInM) const
{
	inliers_dst.clear();
	std::vector<std::pair<float,int> > dists;
	mean_err_dst = 0;
	int size_M = object_points.size();
	if (size_M!=image_points.size())
	{
		return;
	}
	cv::Mat matrix_rotation;
	cv::Rodrigues(rvec_src, matrix_rotation);
	double R[9], T[3], k[8]={0};
	double fu, fv, cx, cy;
	R[0] = matrix_rotation.at<double>(0,0);	//rotation
	R[1] = matrix_rotation.at<double>(0,1);
	R[2] = matrix_rotation.at<double>(0,2);
	R[3] = matrix_rotation.at<double>(1,0);
	R[4] = matrix_rotation.at<double>(1,1);
	R[5] = matrix_rotation.at<double>(1,2);
	R[6] = matrix_rotation.at<double>(2,0);
	R[7] = matrix_rotation.at<double>(2,1);
	R[8] = matrix_rotation.at<double>(2,2);
	T[0] = tvec_src.at<double>(0,0);		//translation
	T[1] = tvec_src.at<double>(1,0);
	T[2] = tvec_src.at<double>(2,0);	
	fu = m_camera_mat.at<float>(0,0);		//camera intrinsic
	fv = m_camera_mat.at<float>(1,1);
	cx = m_camera_mat.at<float>(0,2);
	cy = m_camera_mat.at<float>(1,2);
	//k[0] = m_dist_coef.at<float>(0,0);		//distortion_coefficient
	//k[1] = m_dist_coef.at<float>(1,0);
	//k[2] = m_dist_coef.at<float>(2,0);
	//k[3] = m_dist_coef.at<float>(3,0);


	double X, Y, Z, x, y, z, r2, r4, r6, a1, a2, a3, cdist, icdist2, xd, yd, dis;
	cv::Point2f proj;
	float diff_x, diff_y;
	for (int i=0; i<size_M; i++)
	{		
		X = object_points[i].x;
		Y = object_points[i].y;
		Z = object_points[i].z;
		x = R[0]*X + R[1]*Y + R[2]*Z + T[0];
		y = R[3]*X + R[4]*Y + R[5]*Z + T[1];
		z = R[6]*X + R[7]*Y + R[8]*Z + T[2];
		z = z ? 1./z : 1;
		x *= z; y *= z;

		//r2 = x*x + y*y;
		//r4 = r2*r2;
		//r6 = r4*r2;
		//a1 = 2*x*y;
		//a2 = r2 + 2*x*x;
		//a3 = r2 + 2*y*y;
		//cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
		//icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
		//icdist2 = 1;
		//xd = x*cdist*icdist2 + k[2]*a1 + k[3]*a2;
		//yd = y*cdist*icdist2 + k[2]*a3 + k[3]*a1;
				
		//proj.x = xd*fu + cx;//x*fu + cx;//
		//proj.y = yd*fv + cy;//y*fv + cy;//

		//相机未标定的情况下,忽略m_dist_coef;
		proj.x = x*fu + cx;//x*fu + cx;//
		proj.y = y*fv + cy;//y*fv + cy;//
		diff_x = proj.x - image_points[i].x;
		diff_y = proj.y - image_points[i].y;
		dis = diff_x * diff_x + diff_y * diff_y;
		if (dis>squaredMaxInlierDistInM)
		{
			continue;
		}
		inliers_dst.push_back(i);
		mean_err_dst += dis;
	}



	if (inliers_dst.size()<=4)  //at least the samples should be inliers
	{
		mean_err_dst = 1e9;
	}
	else
	{
		mean_err_dst /= inliers_dst.size();
		mean_err_dst = sqrt(mean_err_dst);
	}

}
///Compute the RootSIFT from SIFT according to Arandjelovic and Zisserman
void ImgIndexKdTreeOpenCV::squareroot_descriptor_space(cv::Mat& descriptors)
{
	// Compute sums for L1 Norm
	cv::Mat sums_vec;
	descriptors = cv::abs(descriptors); //otherwise we draw sqrt of negative vals
	cv::reduce(descriptors, sums_vec, 1 /*sum over columns*/, CV_REDUCE_SUM, CV_32FC1);
	for(unsigned int row = 0; row < descriptors.rows; row++){
		int offset = row*descriptors.cols;
		for(unsigned int col = 0; col < descriptors.cols; col++){
			descriptors.at<float>(offset + col) =
				sqrt(descriptors.at<float>(offset + col) / sums_vec.at<float>(row) /*L1-Normalize*/);
		}
	}
}

void ImgIndexKdTreeOpenCV::proc(INPUT_PARAM src, OUTPUT_PARAM& dst)
{
//	//marker detection
//	cv::Mat rgb_img;
//	vector<float> pose;
//	cvtColor(src.img_gray, rgb_img, CV_GRAY2BGR);
//	//detect and return pose
//	if (m_algo_ar_server_->arDetector(rgb_img, pose)) {
//		//print pose info
////		printf("pose: %f, %f, %f; %f, %f, %f, %f \n", pose[0], pose[1], pose[2],
////				pose[3], pose[4], pose[5], pose[6]);
////		float w = pose[6];
////		float x = pose[3];
////		float y = pose[4];
////		float z = pose[5];
//		float angle_roll, angle_pitch, angle_yaw;
////		http://www.cppblog.com/heath/archive/2009/12/13/103127.html
////		dst.angle_roll[0] = atan2(2*(w*x + y*z), 1-2*(x*x + y*y));
////		dst.angle_pitch[0] = asin(2*(w*y - z*x));
////		dst.angle_yaw[0] = atan2(2*(w*z + z*y), 1-2*(y*y + z*z));
//		dst.loc_x[0] = pose[0];
//		dst.loc_y[0] = pose[1];
//		dst.loc_z[0] = pose[2];
//		dst.angle_roll[0] = pose[3];
//		dst.angle_pitch[0] = pose[4];
//		dst.angle_yaw[0] = pose[5];
//
//		dst.bOK = true;
//		dst.loc_type = 1;
//		dst.confidence[0] = 1;
//		return;
//	}
//	return;
	//LARGE_INTEGER freq;  
	//LARGE_INTEGER start_t, stop_t;  
	//double exe_time;
	//QueryPerformanceFrequency(&freq);  
	//QueryPerformanceCounter(&start_t);  
//0.2ms
	m_kp.clear();
	m_desc.release();

	//QueryPerformanceCounter(&stop_t);  
	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage1 cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[1][0][0] += exe_time;
	//m_time_cnt[1][0][0]++;

	//QueryPerformanceCounter(&start_t);
//100ms
	m_detector_surf->detect(src.img_gray, m_kp);

	//QueryPerformanceCounter(&stop_t);  
	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage2 feat detection cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[2][0][0] += exe_time;
	//m_time_cnt[2][0][0]++;
	//cout<<"detected feature num:"<<m_kp.size()<<endl;


	//QueryPerformanceCounter(&start_t);
//200ms
	m_descriptor_surf->compute(src.img_gray, m_kp, m_desc);
	//QueryPerformanceCounter(&stop_t);  
	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage3 feat desc cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[3][0][0] += exe_time;
	//m_time_cnt[3][0][0]++;
	//QueryPerformanceCounter(&start_t);
//0.5ms
	squareroot_descriptor_space(m_desc);

	//QueryPerformanceCounter(&stop_t);  
	//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
	//cout<<"stage4 desc norm cost:"<<exe_time<<"ms"<<endl;
	//m_time_cost[4][0][0] += exe_time;
	//m_time_cnt[4][0][0]++;
	if (XTION == src.flag)
	{
		getFeatLocation(m_kp, src.img_depth);
		quiryLoc(m_desc, dst);
	}
	else if (CAMERA == src.flag)
	{
//		QueryPerformanceCounter(&start_t);
//0.1ms
		getFeatLocation2D(m_kp);
		//QueryPerformanceCounter(&stop_t);  
		//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
		//cout<<"stage5 cost:"<<exe_time<<"ms"<<endl;
		//m_time_cost[5][0][0] += exe_time;
		//m_time_cnt[5][0][0]++;
		//QueryPerformanceCounter(&start_t);
//300ms
		quiryLoc2D(m_desc, dst);
		//QueryPerformanceCounter(&stop_t);  
		//exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart;  
		//cout<<"stage6 cost:"<<exe_time<<"ms"<<endl;
		//m_time_cost[6][0][0] += exe_time;
		//m_time_cnt[6][0][0]++;
	}
	if (dst.bOK)
	{
		//SYSTEMTIME loc_time;
		//GetLocalTime(&loc_time);
		dst.bFiltered = false;
		m_algo_traj_filter.posFilter(dst, src.cap_time);
	}
	dst.loc_type = 0;
	
}

//b2_0926_biground数据的起点坐标为（876，300）;
//map比例为1mm：0.029像素;
//坐标系：x轴向右（北）为正，z轴向上为正，y轴向前（东）为正;
//输入x,y,z的单位为米;
void ImgIndexKdTreeOpenCV::drawRobotPos(IplImage* img_map, float x, float y, float z, int r, int g, int b, float conf)
{
	//	cvDrawCircle()
	int original_x = 820;//806;
	int original_y = 310;//283;
	int pixel_pos_cols = y * 1000 * 0.029;
	int pixel_pos_rows = x * 1000 * 0.029;
	pixel_pos_cols = original_x - pixel_pos_cols;
	pixel_pos_rows = original_y - pixel_pos_rows;

	int radius = 0;
	int thick = 0;
	if (conf>0.7)
	{
		radius = 5;
		thick = 5;
		cvDrawCircle(img_map, cvPoint(pixel_pos_cols,pixel_pos_rows), radius, cvScalar(b,g,r), thick);	//circle
		cvDrawCircle(img_map, cvPoint(pixel_pos_cols,pixel_pos_rows), 2, cvScalar(b,g,r), 2);			//center
	}
	else
	{
		radius = 50 * (1 - conf) * (1 - conf);
		//		radius = (radius>2) ? radius : 2;
		thick = 10 * conf;
		thick = (thick>1) ? thick : 1;
		cvDrawCircle(img_map, cvPoint(pixel_pos_cols, pixel_pos_rows), radius, cvScalar(b,g,r), thick);
		cvDrawCircle(img_map, cvPoint(pixel_pos_cols, pixel_pos_rows), 2, cvScalar(b,g,r), 2);			//center
	}

	return;

	int band = 3;
	if (pixel_pos_rows>=img_map->height-band || pixel_pos_rows<band || 
		pixel_pos_cols>=img_map->width-band || pixel_pos_cols<band)
	{
		return;
	}
	int offset, offset_y;
	for (int i=-band; i<band; i++)
	{
		offset_y = (pixel_pos_rows + i) * img_map->widthStep;
		for (int j=-band; j<band; j++)
		{
			offset = offset_y + (j + pixel_pos_cols) * 3;
			img_map->imageData[offset] = b;
			img_map->imageData[offset+1] = g;
			img_map->imageData[offset+2] = r;
		}
	}

}

void ImgIndexKdTreeOpenCV::filterDepthImg(cv::Mat img_depth, int winSize)
{
	if (!(winSize<img_depth.rows && winSize<img_depth.cols))
	{
		return;
	}
	winSize /= 2;
	cv::Mat img_temp;// = img_depth;
	img_depth.copyTo(img_temp);
	int start_i,start_j,stop_i, stop_j;

	for (int i=0; i<img_depth.rows; i++)
	{
		unsigned short* data_ptr = img_depth.ptr<unsigned short>(i);
		for (int j=0; j<img_depth.cols; j++)
		{
			if (0==data_ptr[j])
			{
				start_i = max(0, i-winSize);
				stop_i = min(img_temp.rows-1, i+winSize);
				start_j = max(0, j-winSize);
				stop_j = min(img_temp.cols-1, j+winSize);
				int sum_value = 0;
				int cnt = 0;
				for (int m=start_i; m<stop_i; m++)
				{
					for (int n=start_j; n<stop_j; n++)
					{
						if (img_temp.at<unsigned short>(m,n)>0)
						{
							sum_value += img_temp.at<unsigned short>(m,n);
							cnt++;
						}

					}
				}
				if (cnt>0)
				{
					data_ptr[j] = sum_value / cnt;
				}
			}
		}
	}
}
void ImgIndexKdTreeOpenCV::calTimeCost(ofstream& outf)
{
	double cost[10][10][10] = {0};
	for (int i=0; i<10; i++)
	{
		for (int j=0; j<10; j++)
		{
			for(int k=0; k<10; k++)
			{
				if (m_time_cnt[i][j][k]!=0)
				{
					cost[i][j][k] = m_time_cost[i][j][k] / m_time_cnt[i][j][k];
				}
			}
		}
	}
	cout<<"before detection time cost: "<<cost[1][0][0]<<endl;
	cout<<"feat detection time cost: "<<cost[2][0][0]<<endl;
	cout<<"feat desc time cost: "<<cost[3][0][0]<<endl;
	cout<<"desc norm time cost: "<<cost[4][0][0]<<endl;
	cout<<"prepare loc time cost: "<<cost[5][0][0]<<endl;
	cout<<"quiryLoc2D time cost: "<<cost[6][0][0]<<endl;
	cout<<'\t'<<"prepare feat match time cost: "<<cost[6][1][0]<<endl;
	cout<<'\t'<<"feat match time cost: "<<cost[6][2][0]<<endl;
	cout<<'\t'<<"vote loc time cost: "<<cost[6][3][0]<<endl;
	cout<<'\t'<<'\t'<<"prepare Ransac time cost: "<<cost[6][3][1]<<endl;
	cout<<'\t'<<'\t'<<"Ransac time cost: "<<cost[6][3][2]<<endl;
	cout<<'\t'<<'\t'<<"after Ransac time cost: "<<cost[6][3][3]<<endl;


	outf<<"before detection time cost: "<<cost[1][0][0]<<endl;
	outf<<"feat detection time cost: "<<cost[2][0][0]<<endl;
	outf<<"feat desc time cost: "<<cost[3][0][0]<<endl;
	outf<<"desc norm time cost: "<<cost[4][0][0]<<endl;
	outf<<"prepare loc time cost: "<<cost[5][0][0]<<endl;
	outf<<"quiryLoc2D time cost: "<<cost[6][0][0]<<endl;
	outf<<'\t'<<"prepare feat match time cost: "<<cost[6][1][0]<<endl;
	outf<<'\t'<<"feat match time cost: "<<cost[6][2][0]<<endl;
	outf<<'\t'<<"vote loc time cost: "<<cost[6][3][0]<<endl;
	outf<<'\t'<<'\t'<<"prepare Ransac time cost: "<<cost[6][3][1]<<endl;
	outf<<'\t'<<'\t'<<"Ransac time cost: "<<cost[6][3][2]<<endl;
	outf<<'\t'<<'\t'<<"after Ransac time cost: "<<cost[6][3][3]<<endl;

}


void ImgIndexKdTreeOpenCV::matrix2RPY(const Eigen::Matrix4f& T, double& roll, double& pitch, double& yaw)
{
	//unit: radian
	roll = atan2(T(2,1), T(2,2));
	pitch = atan2(-T(2,0), sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2)));
	yaw = atan2(T(1,0), T(0,0));
	//change uint to angel
//	roll = roll / PI * 180;
//	pitch = pitch / PI * 180;
//	yaw = yaw / PI * 180;
}
