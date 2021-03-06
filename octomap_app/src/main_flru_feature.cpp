#include "rgbd2ot.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	if(argc<2)
	{
	    printf("Shoud Use as ./main data_path \n");
	    return 0;
	}

	//using std::map to save all data, idx is the timestamp of the odo
	int s_port = 9009;
	int e_port = 9014;
	vector<std::map<double, string> >vIndex;
	loadAllRgbdIndex(argv[1], s_port, e_port, vIndex);
	
	//data: idx = [t_odo], value=[x, y, z, qx, qy, qz, qw]
	std::map<double, vector<double> > pose_data;

	char poseDataFile[255];
	sprintf(poseDataFile, "%s/gmapping/path_odo.txt", argv[1]);
	readPoseData(poseDataFile, pose_data);

	std::map<double, vector<double> > pose_idx; // [x,y,z,qx,qy,qz,qw,cam1_t, cam2_t,]
	double minTimeItv=0.02; // minimum time interval
	synPoseToRgbd(pose_data, vIndex, minTimeItv, pose_idx);


	//transform from left and right to front
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > vCamToBase; // up-front, up-left, up-right, down-front, down-left, down-right
	Eigen::Affine3f affine;
	pcl::getTransformation (0, -0.61, 0, 0., 0.*D2R, 0., affine);
	vCamToBase.push_back(affine.matrix());
	pcl::getTransformation (-0.13, -0.61, -0.21, 0., -90.*D2R, 0., affine);
	vCamToBase.push_back(affine.matrix());
	pcl::getTransformation (0.16, -0.61, -0.18, 0., 90.*D2R, 0., affine);
	vCamToBase.push_back(affine.matrix());

	pcl::getTransformation (0., 0., 0, 0., 0.*D2R, 0., affine);
	vCamToBase.push_back(affine.matrix());
	pcl::getTransformation (-0.13, 0, -0.21, 0., -90.*D2R, 0., affine);
	vCamToBase.push_back(affine.matrix());
	pcl::getTransformation (0.16, 0, -0.18, 0., 90.*D2R, 0., affine);
	vCamToBase.push_back(affine.matrix());

	/*
	ColorOctreeImpl* m_pOctoTree;
	m_pOctoTree = new ColorOctreeImpl(0.05);
	getOtFromSynData(argv[1], s_port, e_port, vCamToBase, pose_idx, m_pOctoTree);
	*/

	//get features
	FeatOcTree* m_pFeatOcTree;
	m_pFeatOcTree = new FeatOcTree(0.05);

	pointcloud_type::Ptr m_pFeatPcd(new pointcloud_type);

	getFeatOtFromSynData(argv[1], s_port, e_port, vCamToBase, pose_idx, m_pFeatOcTree, m_pFeatPcd);

	std::string filename ("mymap_feature.ot");
	m_pFeatOcTree->write(filename);
	
	string pcdname = "mymap_feature.pcd";
	pcl::io::savePCDFile(pcdname.c_str(), *m_pFeatPcd, true);
	printf("Saved pcd to %s \n", pcdname.c_str());

	//add features to ot

	//save ot
	/*
	char otname[255];
	sprintf(otname, "%s/gmapping/mymap.ot", argv[1]);
	std::ofstream outfile(otname, std::ios_base::out | std::ios_base::binary);
	if (outfile.is_open()){
	    m_pOctoTree->write(outfile);
	}
	outfile.close();
	printf("Saved octomap to %s \n", otname);


	//pcd from ot
	pointcloud_type cloud; 
	ot2pcd(m_pOctoTree, cloud);

	//save pcd
	char pcdname[255];
	sprintf(pcdname, "%s/gmapping/mymap.pcd", argv[1]);
	pcl::io::savePCDFile(pcdname, cloud, true);
	printf("Saved pcd to %s \n", pcdname);
	*/

	return 0;

}

