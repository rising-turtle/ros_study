#include "octomap/FeatOctoMapIO.h"

FeatOctoMapIO::FeatOctoMapIO(void)
{
}

FeatOctoMapIO::~FeatOctoMapIO(void)
{
}
void FeatOctoMapIO::getFeatsFromTreeNode(FeatOcTree& feat_tree, vector<POINT_3D>& pos_dst, vector<cv::Mat>& feat_dst, vector<float>& prob_dst)
{
	feat_dst.clear();
	pos_dst.clear();
	prob_dst.clear();
	int cnt = 0;
	for(FeatOcTree::tree_iterator  it = feat_tree.begin_tree(),
		end=feat_tree.end_tree(); it!= end; ++it)
	{
		if(feat_tree.isNodeOccupied(*it))	
		{			
			
			int num = it->getFeatNum();
			float Prob = it->getOccupancy();

			if (num<=0){
				continue;
			}
			POINT_3D pt_temp;
			pt_temp.x = it.getX();
			pt_temp.y = it.getY();
			pt_temp.z = it.getZ();	
			pos_dst.push_back(pt_temp);		
			FeatOcTreeNode::Feature f;
			it->getFeat(f);
			//if (f.feat_cnt==MAX_FEAT_NUM)
			//{
			//	cnt++;
			//	//cout<<"feat num=10, pos: "<<pt_temp.x<<'\t'<<pt_temp.y<<'\t'<<pt_temp.z<<endl;
			//}
			for (int i=0; i<f.feat_cnt; i++)
			{
				cv::Mat feat_des = cv::Mat(1, FEAT_SIZE, CV_32FC1);
				for(int j=0;j<FEAT_SIZE;j++)
				{
					feat_des.at<float>(0,j) = f.feat_desc[i][j];
				}
				feat_dst.push_back(feat_des);	
				prob_dst.push_back(Prob);
			}
		}
	}
//	cout<<"feat num reached 10 node num: "<<cnt<<endl;
}

void FeatOctoMapIO::getFeatsFromTreeLeaf(FeatOcTree& feat_tree, vector<POINT_3D>& pos_dst, vector<cv::Mat>& feat_dst, vector<float>& prob_dst)
{
	feat_dst.clear();
	pos_dst.clear();
	prob_dst.clear();
	int cnt = 0;
	for(FeatOcTree::leaf_iterator it = feat_tree.begin_leafs(),
		end=feat_tree.end_leafs(); it!= end; ++it)
	{
		if(feat_tree.isNodeOccupied(*it))	
		{			
			cnt++;
			int num = it->getFeatNum();
			float Prob = it->getOccupancy();

			if (num<=0){
				continue;
			}
			POINT_3D pt_temp;
			pt_temp.x = it.getX();
			pt_temp.y = it.getY();
			pt_temp.z = it.getZ();	
			pos_dst.push_back(pt_temp);		
			FeatOcTreeNode::Feature f;
			it->getFeat(f);
	

			for (int i=0; i<f.feat_cnt; i++)
			{
				cv::Mat feat_des = cv::Mat(1, FEAT_SIZE, CV_32FC1);
				for(int j=0;j<FEAT_SIZE;j++)
				{
					feat_des.at<float>(0,j) = f.feat_desc[i][j];
				}
				feat_dst.push_back(feat_des);	
				prob_dst.push_back(Prob);
			}
		}
	}	
}

void FeatOctoMapIO::getFeatsFromRegion(FeatOcTree& feat_tree, point3d pt_min, point3d pt_max, vector<POINT_3D>& pos_dst, vector<cv::Mat>& feat_dst, vector<float>& prob_dst)
{
	feat_dst.clear();
	pos_dst.clear();
	prob_dst.clear();
	int cnt = 0;
//	ColorOcTreeNode::Color nowcolor;

	for(FeatOcTree::leaf_bbx_iterator it = feat_tree.begin_leafs_bbx(pt_min, pt_max),
		end=feat_tree.end_leafs_bbx(); it!= end; ++it)
	{ 
		if(feat_tree.isNodeOccupied(*it))	
		{			
			cnt++;
			int num = it->getFeatNum();
			float Prob = it->getOccupancy();

			if (num<=0){
				continue;
			}
			POINT_3D pt_temp;
			pt_temp.x = it.getX();
			pt_temp.y = it.getY();
			pt_temp.z = it.getZ();	
			pos_dst.push_back(pt_temp);		
			FeatOcTreeNode::Feature f;
			it->getFeat(f);

			for (int i=0; i<f.feat_cnt; i++)
			{
				cv::Mat feat_des = cv::Mat(1, FEAT_SIZE, CV_32FC1);
				for(int j=0;j<FEAT_SIZE;j++)
				{
					feat_des.at<float>(0,j) = f.feat_desc[i][j];
				}
				feat_dst.push_back(feat_des);	
				prob_dst.push_back(Prob);
			}
		}
	}
}

void FeatOctoMapIO::getColorsFromRegion(FeatOcTree& feat_tree, point3d pt_min, point3d pt_max, vector<POINT_3D>& pos_dst, vector<FeatOcTreeNode::Color>& color_dst, vector<float>& prob_dst)
{
	color_dst.clear();
	pos_dst.clear();
	prob_dst.clear();
	int cnt = 0;
	for(FeatOcTree::leaf_bbx_iterator it = feat_tree.begin_leafs_bbx(pt_min, pt_max),
		end=feat_tree.end_leafs_bbx(); it!= end; ++it)
	{ 
		if(feat_tree.isNodeOccupied(*it))	{

			cnt++;

			FeatOcTreeNode::Color nowcolor;
			nowcolor.r = it->getColor().r;
			nowcolor.g = it->getColor().g;
			nowcolor.b = it->getColor().b;
			POINT_3D pt_temp;
			pt_temp.x = it.getX();
			pt_temp.y = it.getY();
			pt_temp.z = it.getZ();	

			pos_dst.push_back(pt_temp);
			color_dst.push_back(nowcolor);

		}
	}
}
void FeatOctoMapIO::getColorsFromTreeLeaf(FeatOcTree& feat_tree, vector<POINT_3D>& pos_dst, vector<FeatOcTreeNode::Color>& color_dst, vector<float>& prob_dst)
{
	color_dst.clear();
	pos_dst.clear();
	prob_dst.clear();
	int cnt = 0;
	for(FeatOcTree::leaf_iterator it = feat_tree.begin_leafs(),
		end=feat_tree.end_leafs(); it!= end; ++it)
	{ 
		if(feat_tree.isNodeOccupied(*it))	{

			cnt++;

			FeatOcTreeNode::Color nowcolor;
			nowcolor.r = it->getColor().r;
			nowcolor.g = it->getColor().g;
			nowcolor.b = it->getColor().b;
			POINT_3D pt_temp;
			pt_temp.x = it.getX();
			pt_temp.y = it.getY();
			pt_temp.z = it.getZ();	

			pos_dst.push_back(pt_temp);
			color_dst.push_back(nowcolor);

		}
	}
}
void FeatOctoMapIO::getColorsFromTreeNode(FeatOcTree& feat_tree, vector<POINT_3D>& pos_dst, vector<FeatOcTreeNode::Color>& color_dst, vector<float>& prob_dst)
{
	color_dst.clear();
	pos_dst.clear();
	prob_dst.clear();
	int cnt = 0;
	for(FeatOcTree::tree_iterator  it = feat_tree.begin_tree(),
		end=feat_tree.end_tree(); it!= end; ++it)
	{ 
		if(feat_tree.isNodeOccupied(*it))	{

			cnt++;

			FeatOcTreeNode::Color nowcolor;
			nowcolor.r = it->getColor().r;
			nowcolor.g = it->getColor().g;
			nowcolor.b = it->getColor().b;
			POINT_3D pt_temp;
			pt_temp.x = it.getX();
			pt_temp.y = it.getY();
			pt_temp.z = it.getZ();	

			pos_dst.push_back(pt_temp);
			color_dst.push_back(nowcolor);

		}
	}
}

void FeatOctoMapIO::updateOctoMapOccupancy(FeatOcTree& feat_tree, Pointcloud pos_cloud_src, point3d pos_cam_src)
{
	feat_tree.insertPointCloud(pos_cloud_src, pos_cam_src);
}

void FeatOctoMapIO::updateFeats(FeatOcTree& feat_tree, vector<cv::Mat> desc_src, vector<POINT_3D> pos_src)
{
	for (int k=0; k<pos_src.size(); k++)
	{
		FeatOcTreeNode* result = feat_tree.search(pos_src[k].x, pos_src[k].y, pos_src[k].z);
		result->addFeat((float*)desc_src[k].data, FEAT_SIZE);
	}	
}

void FeatOctoMapIO::getTreeBorder(FeatOcTree& feat_tree, POINT_3D& pt_min, POINT_3D& pt_max)
{
	double x_min, y_min, z_min, x_max, y_max, z_max;
	feat_tree.getMetricMax(x_max, y_max, z_max);
	feat_tree.getMetricMin(x_min, y_min, z_min);
	pt_max.x = (float) x_max;
	pt_max.y = (float) y_max;
	pt_max.z = (float) z_max;
	pt_min.x = (float) x_min;
	pt_min.y = (float) y_min;
	pt_min.z = (float) z_min;
}

void FeatOctoMapIO::updateTreeColors(FeatOcTree& feat_tree, vector<FeatOcTreeNode::Color> color_src, vector<POINT_3D> pos_src)
{
	for (int k=0; k<pos_src.size(); k++)
	{
		FeatOcTreeNode* result = feat_tree.search(pos_src[k].x, pos_src[k].y, pos_src[k].z);
		result->setColor(color_src[k]);
	}	
	// set inner node colors
	feat_tree.updateInnerOccupancy();
}

void FeatOctoMapIO::saveFeatMapAsFOT(string path_map, string path_FOM, double resolution)
{
	FeatOcTree tree(resolution);
	int en = 1 / resolution;
	ifstream fl_map(path_map.c_str());
	int feat_num = 0;
	int feat_dim = 0;
	fl_map>>feat_num>>feat_dim;
	if (feat_dim != FEAT_SIZE)
	{
		cout<< "feat size is different: "<< feat_dim<<"vs"<<FEAT_SIZE<<endl;
		return;
	}
	cout<<"input feat num: "<<feat_num<<endl;
	float x, y, z;
	float desc[FEAT_SIZE];
	for (int i=0; i<feat_num; i++)
	{

		fl_map>>x>>y>>z;
		for (int j=0; j<FEAT_SIZE; j++)
		{
			fl_map>>desc[j];
		}
		int x_ = floor(x * en + 0.5);
		int y_ = floor(y * en + 0.5);
		int z_ = floor(z * en + 0.5);
		x = 1.0 * x_ / en;
		y = 1.0 * y_ / en;
		z = 1.0 * z_ / en;
		point3d endpoint ((float) x, (float) y, (float) z);
		FeatOcTreeNode* n = tree.updateNode(endpoint, true); 
		n->setColor(101, 101, 101); // set color to red
		n->addFeat(desc, FEAT_SIZE);
	}
	tree.updateInnerOccupancy();

	std::cout << "Writing feat tree to " << path_FOM << std::endl;
	// write color tree
	tree.write(path_FOM);
}

//input: path of FeatOctoMap
//input: read abstractOctree, which should be delete after the usage of featOctoMap
//output: return the read featOctoMap
FeatOcTree* FeatOctoMapIO::readFeatOctoMap(string path_FOM, AbstractOcTree* &read_tree)
{
	cout << "Reading color tree from "<< path_FOM <<"\n";
	read_tree = AbstractOcTree::read(path_FOM);
	cout<<"tree type: "<<read_tree->getTreeType()<<endl;
	cout<<"tree resolution: "<<read_tree->getResolution()<<endl;
	cout<<"tree size: "<<read_tree->size()<<endl;
	FeatOcTree* feat_tree = dynamic_cast<FeatOcTree*>(read_tree);
	return feat_tree;
}

void FeatOctoMapIO::deleteFeatOctoMap(FeatOcTree& feat_tree)
{
	for(FeatOcTree::leaf_iterator it = feat_tree.begin_leafs(),
		end=feat_tree.end_leafs(); it!= end; ++it)
	{
		if(feat_tree.isNodeOccupied(*it))	
		{			
			int num = it->getFeatNum();
			if (num<=0){
				continue;
			}
			it->deleteFeats();
		}
	}	
	feat_tree.clear();
}