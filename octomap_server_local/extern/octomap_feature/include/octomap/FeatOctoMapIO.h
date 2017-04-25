/*
Author: SHEN Hao
Data:	2014.1.13

info: some operation interface for featOctTree
*/

#pragma once
#include "FeatOcTree.h"
#include "opencv2/opencv.hpp"
#include <fstream>
using namespace std;
using namespace octomap;

struct POINT_3D{
	float x;
	float y;
	float z;
};

class FeatOctoMapIO
{
public:
	FeatOctoMapIO(void);
	~FeatOctoMapIO(void);

	//get all features from all nodes
	//input:	featOctoTree
	//output:	point 3D position
	//output:	point descriptor
	//output:	point occupancy probability
	static void getFeatsFromTreeNode(
		FeatOcTree&						feat_tree,
		vector<POINT_3D>&				pos_dst, 
		vector<cv::Mat>&				feat_dst,
		vector<float>&					prob_dst);

	//get all features from leaf nodes, faster than from all nodes
	//input:	featOctoTree
	//output:	point 3D position
	//output:	point descriptor
	//output:	point occupancy probability
	static void getFeatsFromTreeLeaf(
		FeatOcTree&						feat_tree,
		vector<POINT_3D>&				pos_dst, 
		vector<cv::Mat>&				feat_dst,
		vector<float>&					prob_dst);

	//get all features from interest rectangle region
	//input:	featOctoTree
	//input:	pt_min, left and top point of the rectangle region
	//input:	pt_max, right and bottom point of the rectangle region
	//output:	point 3D position
	//output:	point descriptor
	//output:	point occupancy probability
	static void getFeatsFromRegion(
		FeatOcTree&						feat_tree, 
		point3d							pt_min, 
		point3d							pt_max, 
		vector<POINT_3D>&				pos_dst, 
		vector<cv::Mat>&				feat_dst, 
		vector<float>&					prob_dst);


	//get all colors from all nodes
	//input:	featOctoTree
	//output:	point 3D position
	//output:	point color
	//output:	point occupancy probability
	static void getColorsFromTreeNode(
		FeatOcTree&						feat_tree,
		vector<POINT_3D>&				pos_dst, 
		vector<FeatOcTreeNode::Color>&	color_dst,
		vector<float>&					prob_dst);

	//get all colors from leaf nodes, faster than from all nodes, only leaf nodes are extracted
	//input:	featOctoTree
	//output:	point 3D position
	//output:	point color
	//output:	point occupancy probability
	static void getColorsFromTreeLeaf(
		FeatOcTree&						feat_tree,
		vector<POINT_3D>&				pos_dst, 
		vector<FeatOcTreeNode::Color>&	color_dst,
		vector<float>&					prob_dst);

	//get all colors from interest rectangle region
	//input:	featOctoTree
	//input:	pt_min, left and top point of the rectangle region
	//input:	pt_max, right and bottom point of the rectangle region
	//output:	point 3D position
	//output:	point color
	//output:	point occupancy probability
	static void getColorsFromRegion(
		FeatOcTree&						feat_tree, 
		point3d							pt_min, 
		point3d							pt_max, 
		vector<POINT_3D>&				pos_dst, 
		vector<FeatOcTreeNode::Color>&	color_dst, 
		vector<float>&					prob_dst);

	//use the observed point cloud to update the node occupancy in FeatOctoMap
	//input: initial featOctoMap
	//input: observed point cloud
	//input: camera position
	//output: updated featOctoMap
	static void updateOctoMapOccupancy(
		FeatOcTree&						feat_tree, 
		Pointcloud						pos_cloud_src,
		point3d							pos_cam_src);

	// use the observed features to update the features in FeatOctoMap
	//input: initial featOctoMap
	//input: observed feature descriptors
	//input: observed feature 3D position
	//output: updated featuOctoMap
	static void updateFeats(
		FeatOcTree&						feat_tree,
		vector<cv::Mat>					desc_src,
		vector<POINT_3D>				pos_src);

	// use the observed colors to update the colors in FeatOctoMap
	//input: initial featOctoMap
	//input: observed point colors
	//input: observed point 3D position
	//output: updated featuOctoMap
	static void updateTreeColors(
		FeatOcTree&						feat_tree,
		vector<FeatOcTreeNode::Color>	color_src,
		vector<POINT_3D>				pos_src);

	// get the rectangle boundary of FeatOctoMap
	//input: featOctoMap
	//output: left and top point of the rectangle border of tree
	//output: right and bottom point of the rectangle border of tree
	static void getTreeBorder(
		FeatOcTree&						feat_tree,
		POINT_3D&						pt_min,
		POINT_3D&						pt_max);

	// save the original feature map as FeatOctoMap
	//input: input feature map path
	//input: output featOctoMap path
	static void saveFeatMapAsFOT(
		string							path_map, 
		string							path_FOM, 
		double							resolution);

	// read FeatOctoMap from file
	//input: path of FeatOctoMap
	//input: read abstractOctree, which should be delete after the usage of featOctoMap
	//output: return the read featOctoMap
	static FeatOcTree* readFeatOctoMap(
		string							path_FOM,
		AbstractOcTree*					&read_tree);

	// delete FeatOctoMap
	//input: featOctoMap
	static void deleteFeatOctoMap(
		FeatOcTree&						feat_tree);

};
