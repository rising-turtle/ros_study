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
    pcl::PointCloud<point_type>::Ptr src (new pcl::PointCloud<point_type>);
    //read input src cloud
    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    if (reader.read (argv[1], *src) < 0)
    {   
            std::cerr << "Failed to read source input pcd file." << std::endl;
            return (-1);
    }   
    //insert pcd to octree
    ColorOctreeImpl* m_pOctoTree;
    m_pOctoTree = new ColorOctreeImpl(0.05);
    float p[7];
    p[0] = 0;//camTrans.getOrigin().getX();
    p[1] = 0;//camTrans.getOrigin().getY();
    p[2] = 0;//camTrans.getOrigin().getZ();
    p[3] = 0;//camTrans.getRotation().getW();
    p[4] = 0;//camTrans.getRotation().getX();
    p[5] = 0;//camTrans.getRotation().getY();
    p[6] = 0;//camTrans.getRotation().getZ();
    cout<<"Start to insert!"<<endl;
    int max_range = 5;
    m_pOctoTree->insertPointCloud(*src, max_range);
 //    m_pOctoTree->insertPointCloud(*(src), p, max_range);//octomap should use max_range to define which part will be updated
    m_pOctoTree->updateInnerOccupancy();

    FeatOcTree* feat_tree = dynamic_cast<FeatOcTree*>(m_pOctoTree);
    cout<<"OK, finished!"<<endl;

/*
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

	ColorOctreeImpl* m_pOctoTree;
	m_pOctoTree = new ColorOctreeImpl(0.025);
	getOtFromSynData(argv[1], s_port, e_port, vCamToBase, pose_idx, m_pOctoTree);

	//save ot
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

