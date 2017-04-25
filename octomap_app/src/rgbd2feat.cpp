#include "rgbd2feat.h"

void RgbdToFeat::getFeatLocation(vector<cv::KeyPoint> kp, cv::Mat img_depth, vector<POS3D>& pos_3d_dst)
{
	pos_3d_dst.clear();
	for (int i=0; i<(int)kp.size(); i++)
	{			
		float x = kp[i].pt.x;
		float y = kp[i].pt.y;
		//unsigned short z = img_depth.at<unsigned short>(y,x);
		float z = ImgIndexKdTreeOpenCV::getDepthValue(img_depth, x, y, 10);


		float fx = 0.00193897;//525.0;
		float fy = 0.00193638;//525.0;
		float cx = 317.235;//319.5;
		float cy = 239.844;//239.5;
		float factor = 1000; //for the 16-bit PNG files;
		float depth_scale = 1;
		POS3D fea_loc;
		fea_loc.z = z / factor * depth_scale;
		fea_loc.x = (x - cx) * fea_loc.z * fx;
		fea_loc.y = (y - cy) * fea_loc.z * fy;
		//fea_loc(3) = 1;
		pos_3d_dst.push_back(fea_loc);
	}
}

void RgbdToFeat::getFeats(const char* path, const int port, const double t, const Eigen::Matrix4f trans_gl, const Eigen::Matrix4f trans_lc, 
			  vector<cv::Mat> &desc_dst, vector<POS3D> &pos_3d_dst, vector<COLOR> &color_dst, pointcloud_type::Ptr &plane_cloud_1)
{


   vector<cv::KeyPoint>				m_kp;
    cv::Mat								m_desc;
    int									m_img_cnt;
    Eigen::Matrix4f trans_fr_gl = trans_gl * trans_lc;

    char rgbName[255];
    char depthName[255];
    sprintf(rgbName, "%s/raw_data/xtion_%d/rgb/%f.png", path, port, t);
    sprintf(depthName, "%s/raw_data/xtion_%d/depth/%f.png", path, port, t);

    cv::Mat img_rgb = imread(rgbName, -1);
    cv::Mat img_depth = imread(depthName, -1);

    if(!img_rgb.data || !img_depth.data )                              // Check for invalid input
    {
	printf("No Image Found, but Continue! \n");
	return;
    }
    cv::Mat img_gray;
    cv::imshow("rgb", img_rgb);
    cv::waitKey(1);

    cv::cvtColor(img_rgb, img_gray, CV_BGR2GRAY);
    m_kp.clear();
    m_detector_surf->detect(img_gray, m_kp);
    m_descriptor_surf->compute(img_gray, m_kp, m_desc);

    //cout<<"num kp"<<m_kp.size()<<endl;
    ImgIndexKdTreeOpenCV::squareroot_descriptor_space(m_desc);


    //cout<<"whole desc: "<<m_desc<<endl;
    vector<POS3D> pos_3d;
    getFeatLocation(m_kp, img_depth, pos_3d);
    int nSize = pos_3d.size();
    if(nSize<=0)
	return;
    //	ofstream fl_out(txt_path);
    for (int i=0; i<nSize; i++)
    {
	if (pos_3d[i].z<0.3 || pos_3d[i].z>5)
	{
	    continue;
	}
	cv::Mat desc = m_desc.row(i);
	//		cout<<"desc "<<i<<'\t'<<desc<<endl;
	desc_dst.push_back(desc);
	Eigen::Vector4d x_1;
	x_1(0) = pos_3d[i].x;//Í¼Ïñ×ø±êÏµÊÇzÏòÇ°£¬xÏòÓÒ£¬yÏòÏÂ£¬×ªÎªÔªÊý¾Ý×ø±êÏµ;
	x_1(1) = pos_3d[i].y;
	x_1(2) = pos_3d[i].z;
	x_1(3) = 1;
	Eigen::Matrix4d tf_12 = trans_fr_gl.cast<double>();			
	Eigen::Vector3d mu_1_in_2 = (tf_12 * x_1).head<3>();

	POS3D pt;

	pt.x = mu_1_in_2(0);// ÓÉÍ¼Ïñ×ø±êÏµ±äÎªÍ¨ÓÃ×ø±êÏµ;
	pt.y = mu_1_in_2(1);
	pt.z = mu_1_in_2(2);
	/*
	pt.x = mu_1_in_2(0);// ÓÉÍ¼Ïñ×ø±êÏµ±äÎªÍ¨ÓÃ×ø±êÏµ;
	pt.y = mu_1_in_2(2);
	pt.z = -mu_1_in_2(1);
	*/
	pos_3d_dst.push_back(pt);
	COLOR c;
	int offset = 3 * (m_kp[i].pt.y * img_rgb.cols + m_kp[i].pt.x);
	c.b = img_rgb.data[offset];
	c.g = img_rgb.data[offset + 1];
	c.r = img_rgb.data[offset + 2];
	//		fl_out<<pt.x<<'\t'<<pt.y<<'\t'<<pt.z<<'\t'<<(int)c.r<<'\t'<<(int)c.g<<'\t'<<(int)c.b<<endl;
	float* temp_desc = (float*)desc.data;
	//		cout<<"saved feature: "<<endl;
	for (int k=0; k<desc.cols; k++)
	{
	    //			fl_out<<temp_desc[k]<<'\t';
	    //			cout<<temp_desc[k]<<'\t';
	}
	//		cout<<endl;
	//		fl_out<<endl;
	color_dst.push_back(c);

	point_type pt_;

	pt_.x = mu_1_in_2(0);
	pt_.y = mu_1_in_2(1);
	pt_.z = mu_1_in_2(2);
	/*
	pt_.x = mu_1_in_2(0);// ÓÉÍ¼Ïñ×ø±êÏµ±äÎªÍ¨ÓÃ×ø±êÏµ;
	pt_.y = mu_1_in_2(2);
	pt_.z = -mu_1_in_2(1);
	if (pt_.z>2)
	{
	    continue;
	}
	*/
	pt_.b = c.b;
	pt_.g = c.g;
	pt_.r = c.r;
	plane_cloud_1->push_back(pt_);

    }
    	cout<<"feautrue num: "<<color_dst.size()<<endl;
    m_img_cnt++;
}

