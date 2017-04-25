// featOctoMap.cpp : Defines the entry point for the console application.
//

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/FeatOcTree.h>
#include "octomap/FeatOctoMapIO.h"
#include "testing.h"
#include "opencv2/opencv.hpp"
//#include <Windows.h>

using namespace std;
using namespace octomap;

//struct POINT_3D{
//	float x;
//	float y;
//	float z;
//};

void getColorsFromRegion(ColorOcTree& color_tree, point3d pt_min, point3d pt_max, vector<POINT_3D>& pos_dst, vector<ColorOcTreeNode::Color>& c_dst, int& feat_num)
{

	//point3d pt_temp;
//	CvPoint3D32f pt_temp;
	double x, y, z;
	int cnt = 0;
//	vector<FeatOcTreeNode::Feature> feat_all;	
	
	for(ColorOcTree::leaf_bbx_iterator it = color_tree.begin_leafs_bbx(pt_min, pt_max),
		end=color_tree.end_leafs_bbx(); it!= end; ++it)
	{ 
		if(color_tree.isNodeOccupied(*it))	{
			
			cnt++;

			ColorOcTreeNode::Color nowcolor;
			nowcolor.r = it->getColor().r;
			nowcolor.g = it->getColor().g;
			nowcolor.b = it->getColor().b;
			POINT_3D pt_temp;
			pt_temp.x = it.getX();
			pt_temp.y = it.getY();
			pt_temp.z = it.getZ();	

			pos_dst.push_back(pt_temp);
			c_dst.push_back(nowcolor);
			
		}
	}
	feat_num = cnt;

}

void getFeatsFromTreeNode(FeatOcTree& feat_tree)
{
	int cnt = 0;
	for(FeatOcTree::tree_iterator  it = feat_tree.begin_tree(),
		end=feat_tree.end_tree(); it!= end; ++it)
	{
		//manipulate node, e.g.:
		if (cnt%10 == 0)
		{
			std::cout << "Node center: " << it.getCoordinate() << std::endl;
			std::cout << "Node size: " << it.getSize() << std::endl;
			std::cout << "Node value: " << it->getValue() << std::endl;
		}
		
		cnt++;
	}
	cout<<"total node num: "<<cnt<<endl;
}

void getFeatsFromTreeLeaf(FeatOcTree& feat_tree)
{
	int cnt = 0;
	for(FeatOcTree::leaf_iterator it = feat_tree.begin_leafs(),
		end=feat_tree.end_leafs(); it!= end; ++it)
	{
		//manipulate node, e.g.:
		std::cout << "Node center: " << it.getCoordinate() << std::endl;
		std::cout << "Node size: " << it.getSize() << std::endl;
		std::cout << "Node value: " << it->getValue() << std::endl;
		cnt++;
	}
	cout<<"leaf node num: "<<cnt<<endl;
}
void getFeatsFromRegion(FeatOcTree& feat_tree, point3d pt_min, point3d pt_max, vector<POINT_3D>& pos_dst, vector<cv::Mat>& f_dst, vector<int>& feat_num)
{
	int cnt = 0;
	ColorOcTreeNode::Color nowcolor;
	
	for(FeatOcTree::leaf_bbx_iterator it = feat_tree.begin_leafs_bbx(pt_min, pt_max),
		end=feat_tree.end_leafs_bbx(); it!= end; ++it)
	{ 
		if(feat_tree.isNodeOccupied(*it))	
		{			
			cnt++;
		//	OcTreeKey key = it.getKey();
			
			//nowcolor.r = it->getColor().r;
			//nowcolor.g = it->getColor().g;
			//nowcolor.b = it->getColor().b;
			int num = it->getFeatNum();

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
				f_dst.push_back(feat_des);		
			}
			feat_num.push_back(f.feat_cnt);			
		}
	}
//	feat_num = cnt;

}

void print_query_info(point3d query, FeatOcTreeNode* node) {
	if (node != NULL) {
		cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
		cout << "color of node is: " << node->getColor()
			<< endl;    
	}
	else 
		cout << "occupancy probability at " << query << ":\t is unknown" << endl;  
}

void print_query_info(point3d query, ColorOcTreeNode* node) {
	if (node != NULL) {
		cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
		cout << "color of node is: " << node->getColor()
			<< endl;    
	}
	else 
		cout << "occupancy probability at " << query << ":\t is unknown" << endl;  
}

void testColorOctoMap()
{

	double res = 0.02;  // create empty tree with resolution 0.05 (different from default 0.1 for test)
	ColorOcTree tree (0.01);
	// insert some measurements of occupied cells
	for (int x=-20; x<20; x++) {
		for (int y=-20; y<20; y++) {
			for (int z=-20; z<20; z++) {
				point3d endpoint ((float) x*res+0.01f, (float) y*res+0.01f, (float) z*res+0.01f);
				ColorOcTreeNode* n = tree.updateNode(endpoint, true); 
				n->setColor(z*5+101,x*5+101,y*5+101); // set color 
				if (x==y && x==z)
				{
					x = x;
				}
				point3d query ((float) -0.99, (float) -0.99, (float) -0.99);
				ColorOcTreeNode* result = tree.search (query);
				point3d query1 ((float) -0.94, (float) -0.94, (float) -0.94);
				ColorOcTreeNode* result1 = tree.search (query1);			
				
			}
		}
	}

	int x = -10;
	int y = -10;
	int z = -10;
	point3d pt_min((float) x*res+0.01f, (float) y*res+0.01f, (float) z*res+0.01f);
	x = 10;
	y = 10;
	z = 10;
	point3d pt_max((float) x*res+0.01f, (float) y*res+0.01f, (float) z*res+0.01f);
	vector<POINT_3D> pos_roi;
	vector<ColorOcTreeNode::Color> feat_roi;
	int feat_num;
	getColorsFromRegion(tree, pt_min, pt_max, pos_roi, feat_roi, feat_num);
	ofstream fl_out1("roiColors.txt");
	fl_out1<<"pt_min:x, y, z: "<<pt_min.x()<<'\t'<<pt_min.y()<<'\t'<<pt_min.z()<<endl;
	fl_out1<<"pt_max:x, y, z: "<<pt_max.x()<<'\t'<<pt_max.y()<<'\t'<<pt_max.z()<<endl;
	for (int i=0; i<pos_roi.size(); i++)
	{
		fl_out1<<pos_roi[i].x<<'\t'<<pos_roi[i].y<<'\t'<<pos_roi[i].z<<'\t'<<(int)feat_roi[i].r<<'\t'<<(int)feat_roi[i].g<<'\t'<<(int)feat_roi[i].b<<endl;
		
	}
	return;

	ofstream fl_out("log1.txt");
	for (int x=-20; x<20; x++) {
		for (int y=-20; y<20; y++) {
			for (int z=-20; z<20; z++) {
				point3d endpoint ((float) x*res+0.01f, (float) y*res+0.01f, (float) z*res+0.01f);
				//ColorOcTreeNode* n = tree.updateNode(endpoint, true); 
				//n->setColor(z*5+101,x*5+101,y*5+101); // set color 
				//point3d query ((float) -0.99, (float) -0.99, (float) -0.99);
				ColorOcTreeNode* result = tree.search (endpoint);
				//point3d query1 ((float) -0.94, (float) -0.94, (float) -0.94);
				//ColorOcTreeNode* result1 = tree.search (query1);
				//if (x==y && x==z)
				//{
				//	x = x;
				//}
				fl_out<<"x, y, z; r, g, b: "<<x<<'\t'<<y<<'\t'<<z<<'\t';
				if (result)
				{
					ColorOcTreeNode::Color c = result->getColor();
					fl_out<<(int)c.r<<'\t'<<(int)c.g<<'\t'<<(int)c.b<<endl;
				}


			}
		}
	}

	// insert some measurements of free cells
	//for (int x=-30; x<30; x++) {
	//	for (int y=-30; y<30; y++) {
	//		for (int z=-30; z<30; z++) {
	//			point3d endpoint ((float) x*0.02f+2.0f, (float) y*0.02f+2.0f, (float) z*0.02f+2.0f);
	//			ColorOcTreeNode* n = tree.updateNode(endpoint, false); 
	//			n->setColor(255,255,0); // set color to yellow
	//		}
	//	}
	//}

	// set inner node colors
	tree.updateInnerOccupancy();

	cout << endl;


	std::string filename ("simple_color_tree.ot");
	std::cout << "Writing color tree to " << filename << std::endl;
	// write color tree
	EXPECT_TRUE(tree.write(filename));


	// read tree file
	cout << "Reading color tree from "<< filename <<"\n";
	AbstractOcTree* read_tree = AbstractOcTree::read(filename);
	EXPECT_TRUE(read_tree);
	EXPECT_EQ(read_tree->getTreeType().compare(tree.getTreeType()), 0);
	EXPECT_FLOAT_EQ(read_tree->getResolution(), tree.getResolution());
	EXPECT_EQ(read_tree->size(), tree.size());
	ColorOcTree* read_color_tree = dynamic_cast<ColorOcTree*>(read_tree);
	EXPECT_TRUE(read_color_tree);


	cout << "Performing some queries:" << endl;

	{
		point3d query (0., 0., 0.);
		ColorOcTreeNode* result = tree.search (query);
		ColorOcTreeNode* result2 = read_color_tree->search (query);
		std::cout << "READ: ";
		print_query_info(query, result);
		std::cout << "WRITE: ";
		print_query_info(query, result2);
		EXPECT_TRUE(result);
		EXPECT_TRUE(result2);
		EXPECT_EQ(result->getColor(), result2->getColor());
		EXPECT_EQ(result->getLogOdds(), result2->getLogOdds());

		query = point3d(-0.99,-0.99,-0.99);
		result = tree.search (query);
		result2 = read_color_tree->search (query);
		print_query_info(query, result);
		std::cout << "READ: ";
		print_query_info(query, result);
		std::cout << "WRITE: ";
		print_query_info(query, result2);
		EXPECT_TRUE(result);
		EXPECT_TRUE(result2);
		EXPECT_EQ(result->getColor(), result2->getColor());
		EXPECT_EQ(result->getLogOdds(), result2->getLogOdds());

		query = point3d(1.,1.,1.);
		result = tree.search (query);
		result2 = read_color_tree->search (query);
		print_query_info(query, result);
		std::cout << "READ: ";
		print_query_info(query, result);
		std::cout << "WRITE: ";
		print_query_info(query, result2);
		EXPECT_FALSE(result);
		EXPECT_FALSE(result2);

	}

	delete read_tree;
}
void testFeatOctoMap()
{
	double res = 0.02;  // create empty tree with resolution 0.05 (different from default 0.1 for test)
	float res_test = 0.02;
	FeatOcTree tree(res);

	// insert some measurements of occupied cells
	for (int x=-20; x<20; x++) {
		for (int y=-20; y<20; y++) {
			for (int z=-20; z<20; z++) {
				point3d endpoint ((float) x*res_test+0.01f, (float) y*res_test+0.01f, (float) z*res_test+0.01f);
				FeatOcTreeNode* n = tree.updateNode(endpoint, true); 
				n->setColor(z*5+101,x*5+101,y*5+101); // set color to red
				float desc[FEAT_SIZE];
				for (int i=0; i<FEAT_SIZE; i++)
				{
					desc[i] = (x + y + z) + i * 0.1;
				}
				//if (x==-20 && y==-20 && z==-20)
				//{
				//	cout<<"pos x, y, z"<<(float) x*res_test+0.01f<<'\t'<<(float) y*res_test+0.01f<<
				//		'\t'<<(float) z*res_test+0.01f<<endl;
				//	cout<<"input feature: "<<endl;
				//	for (int i=0; i<FEAT_SIZE; i++)
				//	{
				//		cout<<desc[i]<<'\t';
				//	}
				//	cout<<endl;
				//}

				n->addFeat(desc, FEAT_SIZE);
				//n->addFeat(desc, FEAT_SIZE);
				//if (x==y && x==z)
				//{
				//	x = x;
				//}
				//point3d query ((float) -20*res_test+0.01f, (float) -20*res_test+0.01f, (float) -20*res_test+0.01f);
				//FeatOcTreeNode* result = tree.search (query);
				//point3d query1 ((float) -19*res_test+0.01f, (float) -19*res_test+0.01f, (float) -19*res_test+0.01f);
				//FeatOcTreeNode* result1 = tree.search (query1);
				//x = x;
			}
		}
	}
	// set inner node colors
	tree.updateInnerOccupancy();

	vector<POINT_3D> pos_dst;
	vector<cv::Mat> desc_dst;
	vector<float> prob_dst;
	vector<FeatOcTreeNode::Color> color_dst;
	//LARGE_INTEGER  t_bg, t_ed, t_freq;
	//QueryPerformanceCounter(&t_bg);
	//double t_bg = getTime();
	FeatOctoMapIO::getFeatsFromTreeLeaf(tree, pos_dst, desc_dst, prob_dst);
	/*
	QueryPerformanceCounter(&t_ed);
	QueryPerformanceFrequency(&t_freq);
	double cost = 1000.0 * (t_ed.QuadPart - t_bg.QuadPart) / t_freq.QuadPart;
	cout<<"getFeatsFromTreeLeaf:"<<endl;
	cout<<"feat num: "<<desc_dst.size()<<'\t'<<"cost:"<<cost<<"ms"<<endl;

	QueryPerformanceCounter(&t_bg);
	//double t_bg = getTime();
	*/
	FeatOctoMapIO::getFeatsFromTreeNode(tree, pos_dst, desc_dst, prob_dst);
	/*
	QueryPerformanceCounter(&t_ed);
	QueryPerformanceFrequency(&t_freq);
	cost = 1000.0 * (t_ed.QuadPart - t_bg.QuadPart) / t_freq.QuadPart;
	cout<<"getFeatsFromTreeNode:"<<endl;
	cout<<"feat num: "<<desc_dst.size()<<'\t'<<"cost:"<<cost<<"ms"<<endl;

	QueryPerformanceCounter(&t_bg);
	//double t_bg = getTime();
	*/
	FeatOctoMapIO::getColorsFromTreeNode(tree, pos_dst, color_dst, prob_dst);
	/*
	QueryPerformanceCounter(&t_ed);
	QueryPerformanceFrequency(&t_freq);
	cost = 1000.0 * (t_ed.QuadPart - t_bg.QuadPart) / t_freq.QuadPart;
	cout<<"getColorsFromTreeNode:"<<endl;
	cout<<"color num: "<<desc_dst.size()<<'\t'<<"cost:"<<cost<<"ms"<<endl;

	QueryPerformanceCounter(&t_bg);
	//double t_bg = getTime();
	*/
	FeatOctoMapIO::getColorsFromTreeLeaf(tree, pos_dst, color_dst, prob_dst);
	/*
	QueryPerformanceCounter(&t_ed);
	QueryPerformanceFrequency(&t_freq);
	cost = 1000.0 * (t_ed.QuadPart - t_bg.QuadPart) / t_freq.QuadPart;
	cout<<"getColorsFromTreeLeaf:"<<endl;
	cout<<"color num: "<<desc_dst.size()<<'\t'<<"cost:"<<cost<<"ms"<<endl;
	*/

	return;


	Pointcloud pt_clouds;	//the input point clouds, which are used to update the occupancy of octoMap.
	point3d pt_cam;
	tree.insertPointCloud(pt_clouds, pt_cam);
	vector<cv::Mat> feat_src;		//the input features, which are used to update the feature descriptors in map
	vector<POINT_3D> feat_pos;
	for (int k=0; k<feat_pos.size(); k++)
	{
		FeatOcTreeNode* result = tree.search(feat_pos[k].x, feat_pos[k].y, feat_pos[k].z);
		result->addFeat((float*)feat_src[k].data, FEAT_SIZE);
	}	

//	tree.computeUpdate(pt_clouds, pt_cam,)

	double x_min, y_min, z_min, x_max, y_max, z_max;
	tree.getMetricMax(x_max, y_max, z_max);
	tree.getMetricMin(x_min, y_min, z_min);

	//get all nodes from tree
	getFeatsFromTreeNode(tree);
	//get all tree leaf node
	//getFeatsFromTreeLeaf(tree);

	int x = -20;
	int y = -20;
	int z = -20;
	point3d pt_min((float) x*res_test+0.01f, (float) y*res_test+0.01f, (float) z*res_test+0.01f);
	x = 20;
	y = 20;
	z = 20;
	point3d pt_max((float) x*res_test+0.01f, (float) y*res_test+0.01f, (float) z*res_test+0.01f);
	vector<POINT_3D> pos_roi;
	vector<cv::Mat> feat_desc_roi;
	vector<int> feat_num;
//	int feat_num;
	getFeatsFromRegion(tree, pt_min, pt_max, pos_roi, feat_desc_roi, feat_num);
	ofstream fl_out("roiFeats.txt");
	fl_out<<"pt_min:x, y, z: "<<pt_min.x()<<'\t'<<pt_min.y()<<'\t'<<pt_min.z()<<endl;
	fl_out<<"pt_max:x, y, z: "<<pt_max.x()<<'\t'<<pt_max.y()<<'\t'<<pt_max.z()<<endl;
	int cnt = 0;
	for (int i=0; i<pos_roi.size(); i++)
	{
		fl_out<<pos_roi[i].x<<'\t'<<pos_roi[i].y<<'\t'<<pos_roi[i].z<<'\t'<<feat_num[i]<<endl;
		for (int j=0; j<feat_num[i]; j++)
		{
			fl_out<<feat_desc_roi[cnt]<<endl;
			cnt++;
		}
	}
	
	//for (int x=-20; x<20; x++) {
	//	for (int y=-20; y<20; y++) {
	//		for (int z=-20; z<20; z++) {
	//			point3d endpoint ((float) x*res_test+0.01f, (float) y*res_test+0.01f, (float) z*res_test+0.01f);
	//			FeatOcTreeNode* n = tree.updateNode(endpoint, true); 
	//			n->setColor(z*5+101,x*5+101,y*5+101); // set color to red
	//			float desc[FEAT_SIZE];
	//			for (int i=0; i<FEAT_SIZE; i++)
	//			{
	//				desc[i] = (x + y + z) + i * 0.1;
	//			}
	//			if (x==-20 && y==-20 && z==-20)
	//			{
	//				cout<<"input feature: "<<endl;
	//				for (int i=0; i<FEAT_SIZE; i++)
	//				{
	//					cout<<desc[i]<<'\t';
	//				}
	//				cout<<endl;
	//			}

	//			n->addFeat(desc, FEAT_SIZE);
	//			n->addFeat(desc, FEAT_SIZE);
	//			if (x==y && x==z)
	//			{
	//				x = x;
	//			}
	//			point3d query ((float) -20*res_test+0.01f, (float) -20*res_test+0.01f, (float) -20*res_test+0.01f);
	//			FeatOcTreeNode* result = tree.search (query);
	//			point3d query1 ((float) -19*res_test+0.01f, (float) -19*res_test+0.01f, (float) -19*res_test+0.01f);
	//			FeatOcTreeNode* result1 = tree.search (query1);
	//			x = x;
	//		}
	//	}
	//}

	ofstream fl_out3("log2.txt");
	for (int x=-20; x<20; x++) {
		for (int y=-20; y<20; y++) {
			for (int z=-20; z<20; z++) {
				point3d endpoint ((float) x*res_test+0.01f, (float) y*res_test+0.01f, (float) z*res_test+0.01f);
				FeatOcTreeNode* result = tree.search (endpoint);
				fl_out3<<"x, y, z; r, g, b: "<<x<<'\t'<<y<<'\t'<<z<<'\t';
				if (result)
				{
					FeatOcTreeNode::Color c = result->getColor();
					fl_out3<<(int)c.r<<'\t'<<(int)c.g<<'\t'<<(int)c.b<<endl;
				}
			}
		}
	}

	// insert some measurements of free cells
	//for (int x=-30; x<30; x++) {
	//	for (int y=-30; y<30; y++) {
	//		for (int z=-30; z<30; z++) {
	//			point3d endpoint ((float) x*0.02f+2.0f, (float) y*0.02f+2.0f, (float) z*0.02f+2.0f);
	//			FeatOcTreeNode* n = tree.updateNode(endpoint, false); 
	//			n->setColor(255,255,0); // set color to yellow
	//		}
	//	}
	//}

	// get feature from one node

	FeatOcTreeNode::Feature feats;
	tree.getNodeFeat((float) -20*res_test+0.01f, (float) -20*res_test+0.01f, (float) -20*res_test+0.01f, feats);
//	tree.search()
	cout<<"get Feature: "<<endl;
	cout<<feats.feat_cnt<<endl;
	for (int i=0; i<feats.feat_cnt; i++)
	{
		for (int j=0; j<FEAT_SIZE; j++)
		{
			cout<<feats.feat_desc[i][j]<<'\t';
		}
		cout<<endl;
	}


	cout << endl;
	std::string filename ("simple_feat_tree.ot");
	std::cout << "Writing feat tree to " << filename << std::endl;
	// write color tree
	EXPECT_TRUE(tree.write(filename));
	// read tree file
	cout << "Reading color tree from "<< filename <<"\n";
	AbstractOcTree* read_tree = AbstractOcTree::read(filename);
	EXPECT_TRUE(read_tree);
	EXPECT_EQ(read_tree->getTreeType().compare(tree.getTreeType()), 0);
	EXPECT_FLOAT_EQ(read_tree->getResolution(), tree.getResolution());
	EXPECT_EQ(read_tree->size(), tree.size());
	FeatOcTree* read_feat_tree = dynamic_cast<FeatOcTree*>(read_tree);
	EXPECT_TRUE(read_feat_tree);
	cout << "Performing some queries:" << endl;
	{
		point3d query ((float) -10*res_test+0.01f, (float) -10*res_test+0.01f, (float) -10*res_test+0.01f);
		FeatOcTreeNode* result = tree.search (query);
		FeatOcTreeNode* result2 = read_feat_tree->search (query);
		std::cout << "READ: ";
		print_query_info(query, result);
		std::cout << "WRITE: ";
		print_query_info(query, result2);
		EXPECT_TRUE(result);
		EXPECT_TRUE(result2);
		EXPECT_EQ(result->getColor(), result2->getColor());
		EXPECT_EQ(result->getLogOdds(), result2->getLogOdds());

		query = point3d(-1.,-1.,-1.);
		result = tree.search (query);
		result2 = read_feat_tree->search (query);
		print_query_info(query, result);
		std::cout << "READ: ";
		print_query_info(query, result);
		std::cout << "WRITE: ";
		print_query_info(query, result2);
		EXPECT_TRUE(result);
		EXPECT_TRUE(result2);
		EXPECT_EQ(result->getColor(), result2->getColor());
		EXPECT_EQ(result->getLogOdds(), result2->getLogOdds());

		query = point3d(1.,1.,1.);
		result = tree.search (query);
		result2 = read_feat_tree->search (query);
		print_query_info(query, result);
		std::cout << "READ: ";
		print_query_info(query, result);
		std::cout << "WRITE: ";
		print_query_info(query, result2);
		EXPECT_FALSE(result);
		EXPECT_FALSE(result2);

	}
	delete read_tree;
}

void testRealFeatOctoMap()
{
	double res = 0.01;  // create empty tree with resolution 0.05 (different from default 0.1 for test)
	//	ColorOcTree tree (res);
	float res_test = 0.01;
	FeatOcTree tree(res);

	// change feature map to octomap
	ifstream fl_map("../data/92_feature.map");
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
		//set resolution as 0.02m
		int en = 1 / res_test;
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
//	cout<<"begin delete featOctoMap"<<endl;
	
//	cout<<"finished delete featOctoMap"<<endl;

	double x_min, y_min, z_min, x_max, y_max, z_max;
	tree.getMetricMax(x_max, y_max, z_max);
	tree.getMetricMin(x_min, y_min, z_min);
	cout<<"boundary: x:"<<x_min<<"--"<<x_max<<endl;
	cout<<"boundary: y:"<<y_min<<"--"<<y_max<<endl;
	cout<<"boundary: z:"<<z_min<<"--"<<z_max<<endl;
	vector<POINT_3D> pos_dst;
	vector<cv::Mat> desc_dst;
	vector<float> prob_dst;
	vector<FeatOcTreeNode::Color> color_dst;
	//LARGE_INTEGER  t_bg, t_ed, t_freq;
	//QueryPerformanceCounter(&t_bg);
	//double t_bg = getTime();
	FeatOctoMapIO::getFeatsFromTreeLeaf(tree, pos_dst, desc_dst, prob_dst);
	/*
	QueryPerformanceCounter(&t_ed);
	QueryPerformanceFrequency(&t_freq);
	double cost = 1000.0 * (t_ed.QuadPart - t_bg.QuadPart) / t_freq.QuadPart;
	cout<<"getFeatsFromTreeLeaf:"<<endl;
	cout<<"feat num: "<<desc_dst.size()<<'\t'<<"cost:"<<cost<<"ms"<<endl;

	//QueryPerformanceCounter(&t_bg);
	////double t_bg = getTime();
	//FeatOctoMapIO::getFeatsFromTreeNode(tree, pos_dst, desc_dst, prob_dst);
	//QueryPerformanceCounter(&t_ed);
	//QueryPerformanceFrequency(&t_freq);
	//cost = 1000.0 * (t_ed.QuadPart - t_bg.QuadPart) / t_freq.QuadPart;
	//cout<<"getFeatsFromTreeNode:"<<endl;
	//cout<<"feat num: "<<desc_dst.size()<<'\t'<<"cost:"<<cost<<"ms"<<endl;

	QueryPerformanceCounter(&t_bg);
	//double t_bg = getTime();
	*/
	FeatOctoMapIO::getColorsFromTreeNode(tree, pos_dst, color_dst, prob_dst);
	/*
	QueryPerformanceCounter(&t_ed);
	QueryPerformanceFrequency(&t_freq);
	cost = 1000.0 * (t_ed.QuadPart - t_bg.QuadPart) / t_freq.QuadPart;
	cout<<"getColorsFromTreeNode:"<<endl;
	cout<<"color num: "<<desc_dst.size()<<'\t'<<"cost:"<<cost<<"ms"<<endl;

	QueryPerformanceCounter(&t_bg);
	//double t_bg = getTime();
	*/
	FeatOctoMapIO::getColorsFromTreeLeaf(tree, pos_dst, color_dst, prob_dst);
	/*
	QueryPerformanceCounter(&t_ed);
	QueryPerformanceFrequency(&t_freq);
	cost = 1000.0 * (t_ed.QuadPart - t_bg.QuadPart) / t_freq.QuadPart;
	cout<<"getColorsFromTreeLeaf:"<<endl;
	cout<<"color num: "<<desc_dst.size()<<'\t'<<"cost:"<<cost<<"ms"<<endl;
	*/

//	return;
	//query feature
	x = -0.216989;
	y = 5.16605;
	z = 1.40052;
	int en = 1 / res_test;
	int x_ = floor(x * en + 0.5);
	int y_ = floor(y * en + 0.5);
	int z_ = floor(z * en + 0.5);
	x = 1.0 * x_ / en;
	y = 1.0 * y_ / en;
	z = 1.0 * z_ / en;
	point3d query (x, y, z);
	FeatOcTreeNode* result = tree.search (query);
	if (result)
	{
		int num = result->getFeatNum();
		float prob = result->getOccupancy();
		cout<<"pos: "<<x<<'\t'<<y<<'\t'<<z<<"   feat num:"<<num<<"Prob: "<<prob<<endl;
		FeatOcTreeNode::Feature f;
		result->getFeat(f);
		for (int i=0; i<num; i++)
		{
			for (int j=0; j<FEAT_SIZE; j++)
			{
				cout<<f.feat_desc[i][j]<<'\t';
			}
			cout<<endl;
		}
	}
	else
	{
		
		cout<<"find no node  "<<endl;
	}

	x = -0.303935;
	y = 5.18585;
	z = 1.40232;
	en = 1 / res_test;
	x_ = floor(x * en + 0.5);
	y_ = floor(y * en + 0.5);
	z_ = floor(z * en + 0.5);
	x = 1.0 * x_ / en;
	y = 1.0 * y_ / en;
	z = 1.0 * z_ / en;
	point3d query1 (x, y, z);
	FeatOcTreeNode* result1 = tree.search (query1);
	if (result1)
	{
		int num = result1->getFeatNum();
		float prob = result1->getOccupancy();
		cout<<"pos: "<<x<<'\t'<<y<<'\t'<<z<<"   feat num:"<<num<<"Prob: "<<prob<<endl;
		FeatOcTreeNode::Feature f;
		result1->getFeat(f);
		
		for (int i=0; i<num; i++)
		{
			for (int j=0; j<FEAT_SIZE; j++)
			{
				cout<<f.feat_desc[i][j]<<'\t';
			}
			cout<<endl;
		}
	}
	else
	{

		cout<<"find no node  "<<endl;
	}

	std::string filename ("realFeatOctoTree.ot");
	std::cout << "Writing feat tree to " << filename << std::endl;
	// write color tree
	tree.write(filename);
	// read tree file
	cout << "Reading color tree from "<< filename <<"\n";
	//AbstractOcTree* read_tree = AbstractOcTree::read(filename);
	//EXPECT_TRUE(read_tree);
	//EXPECT_EQ(read_tree->getTreeType().compare(tree.getTreeType()), 0);
	//EXPECT_FLOAT_EQ(read_tree->getResolution(), tree.getResolution());
	//EXPECT_EQ(read_tree->size(), tree.size());
	//FeatOcTree* read_feat_tree = dynamic_cast<FeatOcTree*>(read_tree);
	//EXPECT_TRUE(read_feat_tree);
	AbstractOcTree* read_tree = NULL;
	FeatOcTree* read_feat_tree = FeatOctoMapIO::readFeatOctoMap(filename, read_tree);
	


	cout<<"query result from read FeatOctoMap"<<endl;
	point3d query2 (x, y, z);
	FeatOcTreeNode* result2 = read_feat_tree->search (query2);
	if (result2)
	{
		int num = result2->getFeatNum();
		float prob = result2->getOccupancy();
		cout<<"pos: "<<x<<'\t'<<y<<'\t'<<z<<"   feat num:"<<num<<"Prob: "<<prob<<endl;
		FeatOcTreeNode::Feature f;
		result2->getFeat(f);

		for (int i=0; i<num; i++)
		{
			for (int j=0; j<FEAT_SIZE; j++)
			{
				cout<<f.feat_desc[i][j]<<'\t';
			}
			cout<<endl;
		}
	}
	else
	{

		cout<<"find no node  "<<endl;
	}

	//QueryPerformanceCounter(&t_bg);
	//double t_bg = getTime();
	FeatOctoMapIO::getFeatsFromTreeLeaf(*read_feat_tree, pos_dst, desc_dst, prob_dst);
	/*
	QueryPerformanceCounter(&t_ed);
	QueryPerformanceFrequency(&t_freq);
	cost = 1000.0 * (t_ed.QuadPart - t_bg.QuadPart) / t_freq.QuadPart;
	cout<<"getFeatsFromTreeLeaf:"<<endl;
	cout<<"feat num: "<<desc_dst.size()<<'\t'<<"cost:"<<cost<<"ms"<<endl;
	*/

	FeatOctoMapIO::deleteFeatOctoMap(tree);
	FeatOctoMapIO::deleteFeatOctoMap(*read_feat_tree);

	//
	
}

int main(int argc, char** argv) {

//	testColorOctoMap();
//	testFeatOctoMap();
	testRealFeatOctoMap();
	int kkk;
	cin>>kkk;
	return 0;
}
