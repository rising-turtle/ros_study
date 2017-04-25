#ifndef IMAGE_INDEX_KD_TREE_OPENCV_H
#define IMAGE_INDEX_KD_TREE_OPENCV_H

#include "param.h"
//#include "SubmapReader.h"
#include "Eigen/Geometry"
#include <set>
#include "TrajFilter.h"
//#include <ros/ros.h>
//#include "src/ar_server.h"
class ImgIndexKdTreeOpenCV
{
public:
	ImgIndexKdTreeOpenCV(void);

	~ImgIndexKdTreeOpenCV(void);

	void initial(
		string							path_global_map,
		int								feat_size,
		float							thrd);

	void clean();

	void proc(
		INPUT_PARAM						src, 
		OUTPUT_PARAM&					dst);

	static void drawRobotPos(
		IplImage*						img_map, 
		float							x, 
		float							y, 
		float							z, 
		int								r, 
		int								g, 
		int								b, 
		float							conf);

	static void filterDepthImg(
		cv::Mat							img_depth, 
		int								winSize);

	void calTimeCost(ofstream& outf);

	static void squareroot_descriptor_space(
		cv::Mat&						descriptors);

	static float getDepthValue(
		cv::Mat							img_depth, 
		float							x, 
		float							y, 
		int								winSize);

	//get euler angles from affine matrix, angle uint:angle
	static void matrix2RPY(
		const Eigen::Matrix4f&			T,
		double&							roll,
		double&							pitch,
		double&							yaw);



private:
	//vote for the best location
	void quiryLoc(
		cv::Mat							desc,
		OUTPUT_PARAM&					dst);

	void quiryLoc2D(
		cv::Mat							desc, 
		OUTPUT_PARAM&					dst);

	void getFeatLocation(
		vector<cv::KeyPoint>			kp, 
		cv::Mat							img_depth);

	void getFeatLocation2D(
		vector<cv::KeyPoint>			kp);

	void voteBestLoc(
		vector<cv::DMatch>				matches, 
		OUTPUT_PARAM&					dst);

	void voteBestLoc2D(
		vector<cv::DMatch>				matches, 
		OUTPUT_PARAM&					dst);

	Eigen::Matrix4f getTransformationRANSAC(
		std_vector_of_eigen_vector4f	locs_quiry, 
		std_vector_of_eigen_vector4f	locs_map, 
		vector<cv::DMatch>				matches,
		float&							mean_err,
		bool&							flag);

	bool getTransformationRANSAC2D(
		vector<cv::Point3f>				object_points, 
		vector<cv::Point2f>				image_points, 
		cv::Mat&						rvec, 
		cv::Mat&						tvec, 
		double&							mean_err);


	Eigen::Matrix4f getTransformFromMatchesUmeyama(
		std_vector_of_eigen_vector4f	locs_quiry, 
		std_vector_of_eigen_vector4f	locs_map, 
		vector<cv::DMatch>				matches);

	bool getTransformFromMatchesSolvePnP(
		vector<cv::Point3f>				object_points,
		vector<cv::Point2f>				image_points,
		vector<int>						match_index,
		cv::Mat&						trans_R,
		cv::Mat&						trans_T);


	double errorsFunction2(
		const Eigen::Vector4f&			x1,
		const Eigen::Vector4f&			x2,
		const Eigen::Matrix4f&			tf_1_to_2) const;

	void computeInliersAndError(
		const std::vector<cv::DMatch> & all_matches,
		const Eigen::Matrix4f&			transformation,
		std_vector_of_eigen_vector4f	origins,
		std_vector_of_eigen_vector4f	earlier,
		std::vector<cv::DMatch>&		inliers, //pure output var
		double&							mean_error,//pure output var: rms-mahalanobis-distance
		double							squaredMaxInlierDistInM) const;

	void computeInliersAndError2D(
		vector<cv::Point3f>				object_points,
		vector<cv::Point2f>				image_points, 
		cv::Mat							rvec_src,
		cv::Mat							tvec_src, 
		vector<int>&					inliers_dst,
		double&							mean_err_dst,
		double							squaredMaxInlierDistInM) const;

	

	vector<cv::DMatch> sample_matches_prefer_by_distance(
		unsigned int					sample_size, 
		vector<cv::DMatch>&				matches_with_depth);

	vector<int> sample_matches_2D(
		unsigned int					sample_size, 
		unsigned int					total_size);


	void addFeature(
		cv::Mat							desc, 
		int								img_index);

	void addFeature();

	void build();

	void readSubmap(
		string							file_name);

	void readGlobalMap(
		string							file_name);



private:

	float*								m_data;			//ORB�������;
	int									m_data_ptr;		//m_data_orb���β����λ��;
	int*								m_index_feat2Img;	//��ע��ǰ���������ķ�ͼ��;

	cv::FlannBasedMatcher				m_matcher_SURF;
	vector<cv::Mat>						m_feat_pool_SURF;
	int*								m_vote_map;			//ͳ��vote map;
	unsigned int*						m_q;				//�������;
	float*								m_ds;				//��������;
	int									m_max_img_index;	//����ͼ������;
	int									m_feat_size;
	float								m_thrd;				//�����������б���ֵ;
	std_vector_of_eigen_vector4f		m_quiry_locs;		//������location;
	vector<cv::Point2f>					m_quiry_locs_2d;
//	CSubmapReader						m_submap;			//loaded submap;
	std_vector_of_eigen_vector4f		m_map_feat_locs;	//feature location in map
	cv::Mat								m_camera_mat;		//camera internal matrix
	cv::Mat								m_dist_coef;

	cv::SurfFeatureDetector *			m_detector_surf;
	cv::SurfDescriptorExtractor *		m_descriptor_surf;
	vector<cv::KeyPoint>				m_kp;
	cv::Mat								m_desc;
	double								m_time_cost[10][10][10];
	int									m_time_cnt[10][10][10];
	TrajFilter							m_algo_traj_filter;
//	ARServer* 							m_algo_ar_server_;
};

#endif
