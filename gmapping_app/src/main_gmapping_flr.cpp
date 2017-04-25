
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"

#define D2R 0.017453 //pi/180
#define R2D 57.296 //180/pi
#define PI 3.1415926

using namespace std;




void pathCallback(const nav_msgs::PathPtr & path)
{
	ofstream ofs_path("path.txt");
	for(unsigned int i=0;i<path->poses.size();i++)
	{
		ofs_path<<std::fixed<<setprecision(6)<<path->poses[i].header.stamp.toSec()<<" "
				<<path->poses[i].pose.position.x<<" "<<path->poses[i].pose.position.y<<" "<<path->poses[i].pose.position.z<<" "
				<<path->poses[i].pose.orientation.x<<" "<<path->poses[i].pose.orientation.y<<" "<<path->poses[i].pose.orientation.z<<" "
				<<path->poses[i].pose.orientation.w<<endl;

	}
}

void mapCallback()
{

}

int main(int argc, char**argv)
{


	if(argc<2)
	{
		printf("Should Use As ./main file");
		return 0;
	}

	ros::init(argc, argv, "main_gmapping_flr");
	ros::NodeHandle n;

	//save path and map
	//sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	//sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	ros::Subscriber path_sub_ = n.subscribe("/path", 1, pathCallback);

	//TODO: in future the odo data is identical to the gmap,  i will delete this part
	tf::Transform odo2map, odoTrans_rel, odoTrans_pre, odoTrans, odoTrans_first;
	odoTrans_rel.setIdentity();
	odoTrans_pre.setIdentity();
	odoTrans.setIdentity();
	odoTrans_first.setIdentity();
	odo2map.setIdentity();

	float odo2map_r=0.;
	float odo2map_p=0.;
	float odo2map_y=0.*D2R;
	odo2map.setRotation(tf::createQuaternionFromRPY(odo2map_r,odo2map_p,odo2map_y));

	//load data

	char line[1024*16];
	string delim(" ");

	string path = argv[1];
	ifstream inf(path.c_str());
	if(!inf.is_open() ){
		cout<<"failed to open file!"<<endl;
		return 0;
	}

	//loop
	vector<vector<float> > odoAll;
	vector<double> odoTime;

	float sick_minRange = 0.;
	float sick_maxRange = 80.;
	float sick_allPoints = 541.;
	float sick_allAperture = 270.;
	float sick_res = 0.5;
	int sick_frequency = 50;

	//use defined param
	float sick_validPoints = 361;//541.;
	float sick_validAperture = 180;//270.;
	float sick_minAngle = -90*D2R;//-135.*D2R;
	float sick_maxAngle = 90*D2R;//135.*D2R;

	vector<vector<float> > sickAll;
	vector<double> sickTime;
	vector<double> rgbdTime_f;
	vector<double> rgbdTime_l;
	vector<double> rgbdTime_r;

	//ros
	printf("Start Reading: %s \n", path.c_str());
	bool first = true;
	char* p;
	while(inf.good())
	{
	     /*format:odo_t, odo_x, odo_y,odo_angle, laser_t, laser_data1 - laser_data541, 
	      rgbd_front_t, rgbd_front_png, rgbd_left_t, rgbd_left_png, rgbd_right_t, rgbd_right_png
	      */
		//read odo
		inf.getline(line, 1024*16);
		if(inf.eof())
			break;

		//odo
		double t = atof(strtok(line, delim.c_str()));

		vector<float> odo;
		odo.push_back(atof(strtok(NULL, delim.c_str()))/100.);
		odo.push_back(atof(strtok(NULL, delim.c_str()))/100.);
		odo.push_back(0.);
		float theta = atof(strtok(NULL, delim.c_str()));// * M_PI/180. ;
		tf::Quaternion q = tf::createQuaternionFromRPY(0,0,theta);
		odo.push_back(q.getX());
		odo.push_back(q.getY());
		odo.push_back(q.getZ());
		odo.push_back(q.getW());

		tf::Vector3 tr(odo[0],odo[1],odo[2]);
		odoTrans.setRotation(q);
		odoTrans.setOrigin(tr);

		if(first)
		{
			odoTrans_pre = odoTrans;
			odoTrans_first = odoTrans;
			first = false;
		}
		else
		{
			odoTrans_rel = odoTrans_pre.inverse()*odoTrans;
			double dist_rel = odoTrans_rel.getOrigin().getX()*odoTrans_rel.getOrigin().getX() +
					odoTrans_rel.getOrigin().getY()*odoTrans_rel.getOrigin().getY();
			double yaw = tf::getYaw(odoTrans_rel.getRotation());
			//printf("dist %f , yaw %f \n", dist_rel, yaw);
			if(dist_rel > 0.001 || fabs(yaw) > 0.1*D2R)
			{
				odoTrans_pre = odoTrans;
			}
			else
			{
				continue;
			}
		}

		odoTime.push_back( t );
		odoAll.push_back(odo);

		//sick
		sickTime.push_back( atof(strtok(NULL, delim.c_str())) );
		vector<float> sick;
		for(int i=0;i<sick_allPoints;i++)
		{
			sick.push_back(atof(strtok(NULL, delim.c_str()))/1000.);
		}
		if(sick_validPoints < sick_allPoints)
		{
			sick.erase(sick.begin(), sick.begin() + (sick_allPoints - sick_validPoints)/2);;
			sick.erase(sick.end() - (sick_allPoints - sick_validPoints)/2, sick.end());;
		}
		sickAll.push_back(sick);
		rgbdTime_f.push_back(atof(strtok(NULL, delim.c_str())));
		p = strtok(NULL, delim.c_str());
		rgbdTime_l.push_back(atof(strtok(NULL, delim.c_str())));
		p = strtok(NULL, delim.c_str());
		rgbdTime_r.push_back(atof(strtok(NULL, delim.c_str())));
		p = strtok(NULL, delim.c_str());

	}
	inf.close();
	printf("Finish Reading: %d odo-laser-rgbd data \n", sickAll.size());

	//publish odo on topic odom
	tf::TransformBroadcaster odom_broadcaster;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan", 5);
	tf::Transform odo_mea, odo_rel;
	//receive
	tf::TransformListener pose_listener(ros::Duration(10));

	ros::Time current_time;
	ros::Rate r(10);
	unsigned int k=0;

	//save pose
	//ofstream ofs_pose("pose_gmap.txt");
	ofstream ofs_time("rgbd_pair.txt");
	while(n.ok() && k<odoAll.size()){
		printf("Publishing >>> (%d/%d) \n", k, odoAll.size());
		printf("Publishing >>> (%f) \n", odoTime[k]);

		ros::spinOnce();               // check for incoming messages
		current_time= ros::Time::now();//.fromSec(odoTime[k]);// = ros::Time::now();

		//first, we'll publish the transform over tf
		tf::Vector3 tr(odoAll[k][0],odoAll[k][1],odoAll[k][2]);
		tf::Quaternion q(odoAll[k][3],odoAll[k][4],odoAll[k][5],odoAll[k][6]);
		odo_mea.setRotation(q);
		odo_mea.setOrigin(tr);

		odo_rel=odo2map*(odoTrans_first.inverse()*odo_mea)*odo2map.inverse();
		odom_broadcaster.sendTransform(tf::StampedTransform(odo_rel, current_time, "/odom","/base_link"));

		//second, we'll publish the scan info
		sensor_msgs::LaserScan scan;
		scan.header.stamp = current_time;
		scan.header.frame_id = "/laser_frame";
		scan.header.seq = k;
		scan.angle_min = sick_minAngle;
		scan.angle_max = sick_maxAngle;
		scan.angle_increment = sick_validAperture*D2R/ sick_validPoints;
		scan.time_increment = (1 / sick_frequency) / (sick_validPoints);
		scan.range_min = sick_minRange;
		scan.range_max = sick_maxRange;

		scan.ranges.resize(sick_validPoints);
		scan.intensities.resize(sick_validPoints);
		for(unsigned int i = 0; i < sick_validPoints; ++i){
			scan.ranges[i] = sickAll[k][i];
			scan.intensities[i] = 0;
		}
		scan_pub.publish(scan);

		//save time coorespondance
		ofs_time<<std::fixed<<setprecision(6)<<current_time.toSec()<<" "<<rgbdTime_f[k]<<" "<<rgbdTime_l[k]<<" "<<rgbdTime_r[k]<<endl;

		//check pose estimation
		/*
	if (pose_listener.waitForTransform("/map", "/base_link", current_time, ros::Duration(0.1))){

		tf::StampedTransform pose_gmap;
		pose_listener.lookupTransform("/map", "/base_link", current_time, pose_gmap);
		//save
		ofs_pose<<std::fixed<<setprecision(6)<<rgbdTime[k]<<" "
		    <<pose_gmap.getOrigin().getX()<<" "<<pose_gmap.getOrigin().getY()<<" "<<pose_gmap.getOrigin().getZ()<<" "
		    <<pose_gmap.getRotation().getX()<<" "<<pose_gmap.getRotation().getY()<<" "<<pose_gmap.getRotation().getZ()<<" "
		    <<pose_gmap.getRotation().getW()<<endl;
	}
		 */

		//sleep
		k++;
		r.sleep();
	}

	ofs_time.close();
	printf("main_gmapping_zheda: DONE!\n");

	sleep(5);
	//ros::spin();
	return 0;
}
