#include "rgbd2ot.h"
#include "rgbd2feat.h"

#define MAX_XTION_RANGE 3

//data: idx = [t_odo], value=[x, y, z, qx, qy, qz, qw]
bool readPoseData(const char* path, std::map<double, vector<double> > & data)
{
    ifstream inf(path);
    if(!inf.is_open() ){
	cout<<"failed to open file: "<<path<<endl;
	return false;
    }

    double t_odo_cur, t_odo;
    vector<double> v_pose_tmp;
    v_pose_tmp.resize(7);

    char line[255];
    string delim(" ");
    while(inf.good())
    {
	inf.getline(line, 255);
	if(inf.eof())
	    break;
	//
	t_odo_cur = atof(strtok(line,delim.c_str())); // false odo time in current system time

	v_pose_tmp[0] = atof(strtok(NULL,delim.c_str()));
	v_pose_tmp[1] = atof(strtok(NULL,delim.c_str()));
	v_pose_tmp[2] = atof(strtok(NULL,delim.c_str()));
	v_pose_tmp[3] = atof(strtok(NULL,delim.c_str()));
	v_pose_tmp[4] = atof(strtok(NULL,delim.c_str()));
	v_pose_tmp[5] = atof(strtok(NULL,delim.c_str()));
	v_pose_tmp[6] = atof(strtok(NULL,delim.c_str()));

	//real odo timestamp
	t_odo_cur = atof(strtok(NULL, delim.c_str())); // again: false odo time in current time
	t_odo = atof(strtok(NULL, delim.c_str())); // real odo time in current time

	data[t_odo] = v_pose_tmp;
    }

    inf.close();
    printf("Loaded %d pose data \n", data.size());
    return true;
}


void loadAllRgbdIndex(const char* path, const int s_port, const int e_port, vector< std::map<double, string> >& vIndex)
{

    vIndex.clear();

    //int s_port = 9009;
    //int e_port = 9014;
    char rgbdName[255];
    for(int i=s_port;i<=e_port;i++)
    {
	sprintf(rgbdName, "%s/raw_data/xtion_%d/index.txt", path, i);
	printf("Going to load %s \n", rgbdName);
	std::map<double, string> index;
	loadRgbdIndex(rgbdName, index);
	vIndex.push_back(index);

	printf("Loaded cam %d with %d indexes \n", i, index.size());
    }


}

void loadRgbdIndex(const char* path, std::map<double, string>& index)
{
    index.clear();

    ifstream inf(path);

    char* p;
    char line[255];
    string delim("\t ");

    double timestamp;
    string picName;
    while(inf.good())
    {
	inf.getline(line, 255);
	if(inf.eof())
	    break;
	
	timestamp = atof(strtok(line, delim.c_str()));
	picName = strtok(NULL, delim.c_str());

	index[timestamp]=picName;
    }
    
}

void synPoseToRgbd(std::map<double, vector<double> > pose, vector<std::map<double, string> >vIndex, double minTimeItv, 
	std::map<double, vector<double> >& pose_idx)
{
    pose_idx.clear();

    std::map<double, vector<double> >::iterator it_pose;
    for(it_pose=pose.begin(); it_pose!=pose.end(); it_pose++)
    {
	//find closest value
	vector<double> best_idx;
	best_idx.resize( vIndex.size());
	for(int k=0;k<vIndex.size();k++)
	{
	    std::map<double, string >::iterator upper = vIndex[k].lower_bound(it_pose->first);
	    if(upper == vIndex[k].begin() || upper->first == it_pose->first )
	    {
		best_idx[k] = upper->first;
	    }
	    else{
		std::map<double, string >::iterator lower = upper;
		--lower;
		if(upper == vIndex[k].end() || 
			(it_pose->first - lower->first) < (upper->first - it_pose->first) )
		{
		    best_idx[k] = lower->first;
		}
		else
		{
		    best_idx[k] = upper->first;
		}
	    }

	    //check time threshold
	    if(fabs(best_idx[k] - it_pose->first) > minTimeItv)
	    {best_idx[k] = -1;} // -1 means no data can be found
	}

	vector<double> best_pose_idx;
	best_pose_idx = it_pose->second;
	best_pose_idx.insert(best_pose_idx.end(), best_idx.begin(), best_idx.end());

	pose_idx[it_pose->first] = best_pose_idx;

    }

    //save
    ofstream ofs;
    ofs.open("pose.txt");

	double tmp ;
    std::map<double, vector<double> >::iterator it;
    for(it=pose.begin(); it!=pose.end(); it++)
    {
	tmp = it->first;
	ofs<<std::setprecision(6)<<tmp<<" ";
	for(int k=0;k<it->second.size();k++)
	{
	    tmp=it->second[k];
	    ofs<<tmp<<" ";
	}
	ofs<<endl;
    }
    ofs.close();

    ofs.open("pose_idx.txt");
    for(it=pose_idx.begin(); it!=pose_idx.end(); it++)
    {
	tmp = it->first;
	ofs<<std::setprecision(6)<<tmp<<" ";
	for(int k=0;k<it->second.size();k++)
	{
	    tmp=it->second[k];
	    ofs<<tmp<<" ";
	}
	ofs<<endl;
    }
    ofs.close();

}

void getOtFromSynData(const char* path,  
	const int s_port, const int e_port,
	const vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > vCamToBase,
	std::map<double, vector<double> > pose_idx, ColorOctreeImpl* m_pOctoTree)
{
    //setting for ot
    m_pOctoTree->clear();
    m_pOctoTree->setClampingThresMin(0.001);
    m_pOctoTree->setClampingThresMax(0.999);
    m_pOctoTree->setOccupancyThres(0.8);
    m_pOctoTree->setProbHit(0.9);
    m_pOctoTree->setProbMiss(0.4);
    float max_range = MAX_XTION_RANGE;

    //define transform from gmapping pose to cam
    tf::Transform gmap2cam, gmapTrans, camTrans, cam2world;
    gmap2cam.setIdentity();
    gmapTrans.setIdentity();
    camTrans.setIdentity();
    cam2world.setIdentity();

    gmap2cam.setRotation(tf::createQuaternionFromRPY(0., -90.*D2R, 90.*D2R));
    cam2world.setRotation(tf::createQuaternionFromRPY(-90.*D2R, 0., 0.));

    double x, y, z, qx, qy, qz, qw;
    int cnt=0;
    std::map<double, vector<double> >::iterator it_pose;
    for(it_pose=pose_idx.begin(); it_pose!=pose_idx.end(); it_pose++)
    {
	printf("processing (%d / %d)\n", cnt++, pose_idx.size());

	x = it_pose->second[0];
	y = it_pose->second[1];
	z = it_pose->second[2];
	qx = it_pose->second[3];
	qy = it_pose->second[4];
	qz = it_pose->second[5];
	qw = it_pose->second[6];
	tf::Vector3 tr(x,y,z);
	tf::Quaternion q(qx,qy,qz,qw);
	gmapTrans.setRotation(q);
	gmapTrans.setOrigin(tr);
	camTrans = cam2world*gmap2cam*gmapTrans*gmap2cam.inverse();

	pointcloud_type::Ptr pcd_agg (new pcl::PointCloud<point_type>);
	bool first = true;
	for(int k=7;k<it_pose->second.size();k++)
	{
	    printf("syn idx %f\n", it_pose->second[k]);
	    if(it_pose->second[k]>0) // -1 mean invalid
	    {
		int idx=k-7;
		int port = s_port+idx;
		pointcloud_type::Ptr pcd;
		reconstructPcd(path, port, it_pose->second[k], vCamToBase[idx], pcd);

		if(first)
		{
		    *pcd_agg = *pcd;
		    first = false;
		}
		else
		{
		    *pcd_agg += *pcd;
		}
	    }

	}
	printf("pcd agg %d \n", pcd_agg->points.size());

    int cout = 0; 
    char saveF[255]={}; 
    sprintf(saveF, "%dth.pcd", ++cout);
	if(pcd_agg->points.size()>1)
	{
	    //octomap server update
	    //convert pose from ros to octomath, i.e., octo quanterion is different [w,x,y,z]
	    float p[7];
	    p[0] = camTrans.getOrigin().getX();
	    p[1] = camTrans.getOrigin().getY();
	    p[2] = camTrans.getOrigin().getZ();
	    p[3] = camTrans.getRotation().getW();
	    p[4] = camTrans.getRotation().getX();
	    p[5] = camTrans.getRotation().getY();
	    p[6] = camTrans.getRotation().getZ();
        pcl::io::savePCDFile(saveF, *pcd_agg);
	    m_pOctoTree->insertPointCloud(*(pcd_agg), p, max_range);//octomap should use max_range to define which part will be updated
	    m_pOctoTree->updateInnerOccupancy();
	    printf("XXXXX");
	}
    }


}

void getFeatOtFromSynData(const char* path,  
	const int s_port, const int e_port,
	const vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > vCamToBase,
	std::map<double, vector<double> > pose_idx, FeatOcTree* m_pFeatOctoTree, pointcloud_type::Ptr& m_pFeatPcd)
{
    RgbdToFeat rgbd2feat;
    vector<cv::Mat> m_desc_dst;
    vector<POS3D> m_pos_3d_dst;
    vector<COLOR> m_color_dst;

    printf("res %f \n", m_pFeatOctoTree->getResolution());
    ColorOctreeImpl* m_pOctoTree;
    m_pOctoTree = new ColorOctreeImpl(m_pFeatOctoTree->getResolution());
    //setting for ot
    m_pOctoTree->clear();
    m_pOctoTree->setClampingThresMin(0.001);
    m_pOctoTree->setClampingThresMax(0.999);
    m_pOctoTree->setOccupancyThres(0.8);
    m_pOctoTree->setProbHit(0.9);
    m_pOctoTree->setProbMiss(0.4);
    float max_range = MAX_XTION_RANGE;

    //define transform from gmapping pose to cam
    tf::Transform gmap2cam, gmapTrans, camTrans, cam2world;
    gmap2cam.setIdentity();
    gmapTrans.setIdentity();
    camTrans.setIdentity();
    cam2world.setIdentity();

    Eigen::Matrix4f camTrans_eig;

    gmap2cam.setRotation(tf::createQuaternionFromRPY(0., -90.*D2R, 90.*D2R));
    cam2world.setRotation(tf::createQuaternionFromRPY(-90.*D2R, 0., 0.));

    double x, y, z, qx, qy, qz, qw;
    int cnt=0;
    std::map<double, vector<double> >::iterator it_pose;
    for(it_pose=pose_idx.begin(); it_pose!=pose_idx.end(); it_pose++)
    {
	/*
	if(cnt>5)
	    break;
	    */

	printf("processing (%d / %d)\n", cnt++, pose_idx.size());

	x = it_pose->second[0];
	y = it_pose->second[1];
	z = it_pose->second[2];
	qx = it_pose->second[3];
	qy = it_pose->second[4];
	qz = it_pose->second[5];
	qw = it_pose->second[6];
	tf::Vector3 tr(x,y,z);
	tf::Quaternion q(qx,qy,qz,qw);
	gmapTrans.setRotation(q);
	gmapTrans.setOrigin(tr);
	camTrans = cam2world*gmap2cam*gmapTrans*gmap2cam.inverse();

	Eigen::Affine3d affine;
	tf::transformTFToEigen(camTrans, affine);
	camTrans_eig = affine.matrix().cast<float>();

	pointcloud_type::Ptr pcd_agg (new pcl::PointCloud<point_type>);
	bool first = true;
	for(int k=7;k<it_pose->second.size();k++)
	{
	    if(it_pose->second[k]>0) // -1 mean invalid
	    {
		int idx=k-7;
		int port = s_port+idx;
		rgbd2feat.getFeats(path, port, it_pose->second[k], camTrans_eig, vCamToBase[idx], m_desc_dst, m_pos_3d_dst, m_color_dst, m_pFeatPcd);

		/*
		pointcloud_type::Ptr pcd;
		reconstructPcd(path, port, it_pose->second[k], vCamToBase[idx], pcd);

		if(first)
		{
		    *pcd_agg = *pcd;
		    first = false;
		}
		else
		{
		    *pcd_agg += *pcd;
		}
		*/
	    }

	}

	/*
	printf("pcd agg %d \n", pcd_agg->points.size());
	if(pcd_agg->points.size()>1)
	{
	    //octomap server update
	    //convert pose from ros to octomath, i.e., octo quanterion is different [w,x,y,z]
	    float p[7];
	    p[0] = camTrans.getOrigin().getX();
	    p[1] = camTrans.getOrigin().getY();
	    p[2] = camTrans.getOrigin().getZ();
	    p[3] = camTrans.getRotation().getW();
	    p[4] = camTrans.getRotation().getX();
	    p[5] = camTrans.getRotation().getY();
	    p[6] = camTrans.getRotation().getZ();
	    m_pOctoTree->insertPointCloud(*(pcd_agg), p, max_range);//octomap should use max_range to define which part will be updated
	    m_pOctoTree->updateInnerOccupancy();
	}
	*/
    }

    //convert colorOctree to featureOctree
    //ColorOcTree* color_tree = dynamic_cast<ColorOcTree*>(m_pOctoTree);
    //FeatOcTree* feat_tree = dynamic_cast<FeatOcTree*>(color_tree);
    //FeatOcTree* feat_tree = dynamic_cast<FeatOcTree*>(m_pOctoTree);
    //m_pFeatOctoTree->swapContent(*feat_tree);

    int feat_num = m_desc_dst.size();
    for (int i=0; i<feat_num; i++)
    {
	printf("process <%d/%d> \n", i, feat_num);
	float x=m_pos_3d_dst[i].x;
	float y=m_pos_3d_dst[i].y;
	float z=m_pos_3d_dst[i].z;
	int en = 1 / m_pFeatOctoTree->getResolution();
	int x_ = floor(x * en + 0.5);
	int y_ = floor(y * en + 0.5);
	int z_ = floor(z * en + 0.5);
	x = 1.0 * x_ / en;
	y = 1.0 * y_ / en;
	z = 1.0 * z_ / en;
	point3d endpoint ((float) x, (float) y, (float) z);
	FeatOcTreeNode* n = m_pFeatOctoTree->updateNode(endpoint, true); 
	n->setColor(m_color_dst[i].r, m_color_dst[i].g, m_color_dst[i].b); // set color to red
	n->addFeat((float*)m_desc_dst[i].data, FEAT_SIZE);
    }

    delete m_pOctoTree;

}


void reconstructPcd(const char* path, const int port, const double timestamp, 
	const Eigen::Matrix4f trans, pointcloud_type::Ptr& pcd)
{

    char rgbName[255];
    char depthName[255];
    sprintf(rgbName, "%s/raw_data/xtion_%d/rgb/%f.png", path, port, timestamp);
    sprintf(depthName, "%s/raw_data/xtion_%d/depth/%f.png", path, port, timestamp);

    cv::Mat rgb_img = imread(rgbName, -1);
    cv::Mat depth_img = imread(depthName, -1);

    if(!rgb_img.data || !depth_img.data )                              // Check for invalid input
    {
	printf("No Image Found, but Continue! \n");
	return;
    }

    //convert depth_img from 16uc1 to 32fc1
    cv::Mat float_img;
    depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
    depth_img = float_img;

    //reconstruct
    pcd =pointcloud_type::Ptr( createXYZRGBPointCloud(depth_img,rgb_img));
    pcl::transformPointCloud ( *pcd, *pcd, trans);
    printf("XX pcd size %d \n", pcd->points.size());

}


void ot2pcd(const ColorOctreeImpl* m_pOctoTree, pointcloud_type& cloud)
{
    for(octomap::ColorOcTree::leaf_iterator it = m_pOctoTree->begin_leafs(), end=m_pOctoTree->end_leafs(); it!= end; ++it)
    {
	if(m_pOctoTree->isNodeOccupied(*it))
	{
	    octomap::point3d pt = it.getCoordinate();
	    octomap::ColorOcTreeNode::Color color = (*it).getColor();
	    point_type color_pt(color.r, color.g, color.b);
	    color_pt.x = pt(0); color_pt.y = pt(1); color_pt.z = pt(2);
	    cloud.points.push_back(color_pt);
	}
    }


}


// depth_img must be 32fc1
pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_img, const cv::Mat& rgb_img)
{

	pointcloud_type* cloud (new pointcloud_type() );
	cloud->is_dense         = false; //single point of view, 2d rasterized NaN where no depth value was found

	//xtion 
	float fx = 1./ 577.24;//cam_info->K[0]; //(cloud->width >> 1) - 0.5f;
	float fy = 1./ 580.48;//cam_info->K[4]; //(cloud->width >> 1) - 0.5f;
	float cx = 302.49;//cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
	float cy = 198.71;//cam_info->K[5]; //(cloud->width >> 1) - 0.5f;
	int data_skip_step = 1;
	if(depth_img.rows % data_skip_step != 0 || depth_img.cols % data_skip_step != 0){
		ROS_WARN("The parameter cloud_creation_skip_step is not a divisor of the depth image dimensions. This will most likely crash the program!");
	}
	cloud->height = ceil(depth_img.rows / static_cast<float>(data_skip_step));
	cloud->width = ceil(depth_img.cols / static_cast<float>(data_skip_step));
	int pixel_data_size = 3;
	bool encoding_bgr = true;
	//Assume BGR
	//char red_idx = 2, green_idx = 1, blue_idx = 0;
	//Assume RGB
	char red_idx = 0, green_idx = 1, blue_idx = 2;
	if(rgb_img.type() == CV_8UC1) pixel_data_size = 1;
	else if(encoding_bgr) { red_idx = 2; blue_idx = 0; }

	unsigned int color_row_step, color_pix_step, depth_pix_step, depth_row_step;
	color_pix_step = pixel_data_size * (rgb_img.cols / cloud->width);
	color_row_step = pixel_data_size * (rgb_img.rows / cloud->height -1 ) * rgb_img.cols;
	depth_pix_step = (depth_img.cols / cloud->width);
	depth_row_step = (depth_img.rows / cloud->height -1 ) * depth_img.cols;

	cloud->points.resize (cloud->height * cloud->width);

	// depth_img already has the desired dimensions, but rgb_img may be higher res.
	int color_idx = 0 * color_pix_step - 0 * color_row_step, depth_idx = 0; //FIXME: Hack for hard-coded calibration of color to depth
	double depth_scaling = 1.;
	float max_depth = -1;//MAX_XTION_RANGE;
	float min_depth = -1;//MIN_XTION_RANGE;
	if(max_depth < 0.0) max_depth = std::numeric_limits<float>::infinity();

	pointcloud_type::iterator pt_iter = cloud->begin();
	for (int v = 0; v < (int)rgb_img.rows; v += data_skip_step, color_idx += color_row_step, depth_idx += depth_row_step)
	{
		for (int u = 0; u < (int)rgb_img.cols; u += data_skip_step, color_idx += color_pix_step, depth_idx += depth_pix_step, ++pt_iter)
		{
			if(pt_iter == cloud->end()){
				break;
			}
			point_type& pt = *pt_iter;
			if(u < 0 || v < 0 || u >= depth_img.cols || v >= depth_img.rows){
				pt.x = std::numeric_limits<float>::quiet_NaN();
				pt.y = std::numeric_limits<float>::quiet_NaN();
				pt.z = std::numeric_limits<float>::quiet_NaN();
				continue;
			}

			float Z = depth_img.at<float>(depth_idx) * depth_scaling;

			// Check for invalid measurements
			if (!(Z >= min_depth)) //Should also be trigger on NaN//std::isnan (Z))
			{
				pt.x = (u - cx) * 1.0 * fx; //FIXME: better solution as to act as at 1meter?
				pt.y = (v - cy) * 1.0 * fy;
				pt.z = std::numeric_limits<float>::quiet_NaN();
			}
			else // Fill in XYZ
			{
				pt.x = (u - cx) * Z * fx;
				pt.y = (v - cy) * Z * fy;
				pt.z = Z;
			}
			// Fill in color
			RGBValue color;
			if(color_idx > 0 && color_idx < rgb_img.total()*color_pix_step){ //Only necessary because of the color_idx offset
				if(pixel_data_size == 3){
					color.Red   = rgb_img.at<uint8_t>(color_idx + red_idx);
					color.Green = rgb_img.at<uint8_t>(color_idx + green_idx);
					color.Blue  = rgb_img.at<uint8_t>(color_idx + blue_idx);
				} else {
					color.Red   = color.Green = color.Blue  = rgb_img.at<uint8_t>(color_idx);
				}
				color.Alpha = 0;
				pt.rgb = color.float_value;
			}
		}
	}

	return cloud;
}
