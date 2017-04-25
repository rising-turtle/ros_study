
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

#include "std_msgs/Bool.h"

#define D2R 0.017453 //pi/180
#define R2D 57.296 //180/pi
#define PI 3.1415926

using namespace std;

bool g_ack_syn = false; 
bool g_save_path_syn = false;


void ackCallback(const std_msgs::BoolPtr& ack)
{
    g_ack_syn = ack->data;
    cout<<"gmapping_query.cpp: get ack !"<<endl;
}

void savePathCallback(const std_msgs::BoolPtr& ack)
{
    g_save_path_syn = ack->data; 
    cout<<"gmapping_query.cpp: save Path succeed, exist!"<<endl;
}

void pathCallback(const nav_msgs::PathPtr & path)
{
    ros::NodeHandle private_nh("~");
    string save_path;
    if(!private_nh.getParam("save_path", save_path))
        save_path = "path.txt";
    else
        save_path = save_path + "/path.txt"; 
	ofstream ofs_path(save_path.c_str());
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
		printf("Should Use As ./main odo_laser.txt");
		return 0;
	}

	ros::init(argc, argv, "main_gmapping_query");
	ros::NodeHandle n;

	//save path and map
	//sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	//sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	ros::Subscriber path_sub_ = n.subscribe("/path", 1, pathCallback);

    // set this query requirement for ack and syn 
    ros::Subscriber ack_sub_ = n.subscribe("/ack", 1, ackCallback);
    ros::Publisher syn_pub_ = n.advertise<std_msgs::Bool>("/syn", 1);
 
    // set save_path query 
    ros::Subscriber path_save_ack_sub_ = n.subscribe("/save_path_ack", 1, savePathCallback);
    ros::Publisher path_save_syn_pub_ = n.advertise<std_msgs::Bool>("/save_path_syn", 1);

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
    
    // save odo 
    vector<vector<float> > odoSaveAll; 
    vector<float> odoSave;

	float sick_minRange = 0.;
	float sick_maxRange = 80.;
	int sick_allPoints = 541;
	float sick_allAperture = 270.;
	float sick_res = 0.5;
	int sick_frequency = 50;

	//use defined param
	int sick_validPoints = 361;//541.;
	float sick_validAperture = 180;//270.;
	float sick_minAngle = -90*D2R;//-135.*D2R;
	float sick_maxAngle = 90*D2R;//135.*D2R;

	vector<vector<float> > sickAll;
	vector<double> sickTime;

   // whether save locally 
    bool save_gt_ ;
    bool save_odo_;
    bool publish_data_;
    int publish_num_;
    
    ros::NodeHandle private_nh("~");
    private_nh.param("sick_points", sick_allPoints, 541);
    private_nh.param("sick_valid_points", sick_validPoints, 181);
    // private_nh.param("save_gt", save_gt_, true);
    private_nh.param("save_odo", save_odo_, true);
    private_nh.param("publish_data", publish_data_, true);
    private_nh.param("publish_num", publish_num_, -1);

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
        
        odoSave = odo;
        odoSave[2] = theta;

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
        odoSaveAll.push_back(odoSave);

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
	
	}
	inf.close();
	printf("Finish Reading: %d odo-laser-rgbd data \n", sickAll.size());

    if(publish_data_)
    {
        //publish odo on topic odom
        tf::TransformBroadcaster odom_broadcaster;
        ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan", 5);
        tf::Transform odo_mea, odo_rel;
        //receive
        tf::TransformListener pose_listener(ros::Duration(10));

        ros::Time current_time;
        ros::Rate r(10);
        unsigned int k=0;

        // first build connection 
        std_msgs::BoolPtr ok(new std_msgs::Bool);
        ok->data = true;
        while(!g_ack_syn)
        {
            cout<<"gmapping_query.cpp: send SYN!"<<endl;
            syn_pub_.publish(ok); 
            ros::spinOnce();
            r.sleep();
        }

        //save pose
        //ofstream ofs_pose("pose_gmap.txt");
        ofstream ofs_time("odo_pair.txt");
        int count = 0; 
        while(n.ok() && k<odoAll.size())
        {
            // wait for ack signal
            if(!g_ack_syn)
            {
                ros::spinOnce();               // check for incoming messages
                r.sleep();
                /*if(++count > 20) // maybe blocked
                {
                    g_ack_syn = true;
                }*/ 
                continue;
            }
            count = 0;
            g_ack_syn = false;
            printf("Publishing >>> (%d/%d) \n", k, odoAll.size());
            printf("Publishing >>> (%f) \n", odoTime[k]);

            ros::spinOnce();               // check for incoming messages
            current_time = ros::Time::now(); //.fromSec(odoTime[k]);// = ros::Time::now();

            //first, we'll publish the transform over tf
            tf::Vector3 tr(odoAll[k][0],odoAll[k][1],odoAll[k][2]);
            tf::Quaternion q(odoAll[k][3],odoAll[k][4],odoAll[k][5],odoAll[k][6]);
            odo_mea.setRotation(q);
            odo_mea.setOrigin(tr);

            odo_rel=odo2map*(odoTrans_first.inverse()*odo_mea)*odo2map.inverse();
            odom_broadcaster.sendTransform(tf::StampedTransform(odo_rel, current_time, "/odom","/base_link"));

            //second, we'll publish the scan info
            sensor_msgs::LaserScan scan;
            scan.header.stamp = current_time;//.fromSec(sickTime[k]);
            scan.header.frame_id = "/laser_frame";
            scan.header.seq = k;
            scan.angle_min = sick_minAngle;
            scan.angle_max = sick_maxAngle;
            scan.angle_increment = sick_validAperture*D2R/ (float)(sick_validPoints);
            scan.time_increment = (1 / sick_frequency) / (float)(sick_validPoints);
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
            ofs_time<<std::fixed<<setprecision(6)<<current_time.toSec()<<" "<<odoTime[k]<<endl;

            odoTime[k] = current_time.toSec(); // this is for compare

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
            if( publish_num_ >0 && k> publish_num_)
            {
                break;
            }
        }

        ofs_time.close();
        while(!g_save_path_syn && n.ok())
        {
            // cout<<"B2_query.cpp: send save_path SYN!"<<endl;
            path_save_syn_pub_.publish(ok); 
            ros::spinOnce();
            r.sleep();
        }
        printf("main_gmapping: DONE!\n");
    }

    if(save_odo_)
    {
        cout<<"Try to save odo in B2_odo.txt"<<endl; 
        ofstream odo("B2_odo.txt"); 
        for(int i=0; i<odoSaveAll.size(); i++)
        {
            odo<<std::fixed<<odoTime[i]<<" "<<odoSaveAll[i][0]<<" "<<odoSaveAll[i][1]<<" "<<odoSaveAll[i][2]<<endl;
        }

    }

	sleep(5);
	//ros::spin();
	return 0;
}
