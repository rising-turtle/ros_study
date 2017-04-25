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

	// ros init
	ros::init(argc, argv, "main_RgbdToOctoGmapping_flr");
	ros::NodeHandle nh;

	// get current CameraInfo data
	camera_info_manager::CameraInfoManager depthInfoMgr(nh, "depth");
	sensor_msgs::CameraInfoPtr 	depthInfo(new sensor_msgs::CameraInfo(depthInfoMgr.getCameraInfo()));

	//using std::map to save all data, idx is the timestamp of the odo
	//data: idx=[t_odo], value=[t_front, t_left, t_right]
	std::map<double, vector<double> > syn_data;
	
	//data: idx = [t_odo], value=[x, y, z, qx, qy, qz, qw]
	std::map<double, vector<double> > pose_data;

	char poseDataFile[255];
	sprintf(poseDataFile, "%s/gmapping/path_odo.txt", argv[1]);
	readPoseData(poseDataFile, pose_data);

	char synDataFile[255];
	sprintf(synDataFile, "%s/odo_laser_front_left_right.txt", argv[1]);
	readSynData(synDataFile, syn_data);

	//reconstruct ot
	ColorOctreeImpl* m_pOctoTree;
	m_pOctoTree = new ColorOctreeImpl(0.05);
	reconstructOt(argv[1], depthInfo, pose_data, syn_data, m_pOctoTree);

	//save ot
	string fname = "result.ot";
	std::ofstream outfile(fname.c_str(), std::ios_base::out | std::ios_base::binary);
	if (outfile.is_open()){
	    m_pOctoTree->write(outfile);
	}
	outfile.close();
	printf("Saved octomap to %s \n", fname.c_str());


	//pcd from ot
	pointcloud_type cloud; 
	ot2pcd(m_pOctoTree, cloud);

	//save pcd
	fname = "result.pcd";
	pcl::io::savePCDFile(fname.c_str(), cloud, true);
	printf("Saved pcd to %s \n", fname.c_str());

	return 0;

}

