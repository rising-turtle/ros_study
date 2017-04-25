#include "interporlation.h"
#include <iostream>
#include <fstream>
#include "tf/tf.h"

#define DEBUG 0
#define NUM  5

namespace {
    void printV(vector<double> & t, string name)
    {
        cout<<name<<" size: "<<t.size()<<endl;
        for(int i=0; i<t.size(); i++)
            cout<<t[i]<<endl;
    }   
}


CInterporlation::CInterporlation(unsigned int steps) : 
m_nsteps(steps)
{
}

CInterporlation::~CInterporlation(){}

void CInterporlation::getAllData(ifstream& inf, vector<vector<double> >& pose, vector<double>& time_1, vector<double>& time_2)
{
    char line[4096]; 
    string delim(" \t");
    int pose_size = 7;
    vector<double> p;
    p.resize(pose_size);
    while(inf.getline(line, 4096))
    {
        double t1 = atof(strtok(line, delim.c_str()));
        for(int i=0; i< pose_size; i++)
        {
            p[i] = atof(strtok(NULL, delim.c_str()));
        }
        strtok(NULL, delim.c_str());
        double t2 = atof(strtok(NULL, delim.c_str()));
        pose.push_back(p);
        time_1.push_back(t1);
        time_2.push_back(t2);
    } 
}

bool CInterporlation::skip_interpolate(vector<double>& p1, vector<double>& p2)
{
    tf::Quaternion q1(p1[3], p1[4], p1[5], p1[6]);
    tf::Quaternion q2(p2[3], p2[4], p2[5], p2[6]);
    double yaw1 = tf::getYaw(q1);
    double yaw2 = tf::getYaw(q2);
    double abs_delta = fabs(yaw2 - yaw1);
    if(abs_delta >= M_PI)
        abs_delta = 2*M_PI - abs_delta;
    if(abs_delta >= D2R(SKIP_ANGLE)) return true; 
    return false;
}

bool CInterporlation::interpolate(string input_f, string suffix)
{
    ifstream inf(input_f.c_str());
    if(!inf.is_open())
    {
        cout<<"failed to open inputf: "<<input_f<<endl;
        return false;
    }
    vector< vector<double> > poses; 
    vector<double> vt1; 
    vector<double> vt2; 
    getAllData(inf, poses, vt1, vt2);

    stringstream outfs; 
    unsigned int loc = input_f.find_last_of(".");
    string subs = input_f.substr(0, loc);
    if(suffix == "")
        outfs<<subs<<"_interpolate.txt";
    else
        outfs<<suffix;
    
    ofstream outf(outfs.str().c_str());
    vector<double> t1_delta(m_nsteps);
    vector<double> t2_delta(m_nsteps);
    vector< vector<double> > pose_delta;
    int i;
    ofstream skip_record("skip_record.txt");
    for(i=0; i<poses.size()-1; i++)
    {
        write_log(outf, vt1[i], vt2[i], poses[i]);
        if(skip_interpolate(poses[i], poses[i+1]))
        {
            skip_record<<i<<"\t"<<i+1<<endl;
            continue;
        }
        interporlate_scalar(vt1[i], vt1[i+1], t1_delta);
        interporlate_scalar(vt2[i], vt2[i+1], t2_delta);
        interporlate_pose(poses[i], poses[i+1], pose_delta);
        if(DEBUG)
        {
            // printV(t1_delta, "t1_delta");
            // printV(t2_delta, "t2_delta");
        }
        for(int j=0; j<m_nsteps; j++)
        {
            write_log(outf, t1_delta[j], t2_delta[j], pose_delta[j]);  
        }    

        if(DEBUG && i>=NUM) return true;
    }
    
    write_log(outf, vt1[i], vt2[i], poses[i]);
    return true;
}

void CInterporlation::write_log(ofstream& outf, double t1, double t2, vector<double>& pose)
{
    outf<<std::fixed<<setprecision(6)<<t1<<"\t";
    for(int i=0; i<pose.size();i++)
        outf<<pose[i]<<"\t";
    outf<<t1<<"\t"<<t2<<endl;
}

void CInterporlation::interporlate_angle(double a1, double a2, vector<double>& values)
{
    double factor = 1.;
    double incr_  = 0;
    double abs_delta = fabs(a2-a1);
    if(abs_delta >= M_PI)
    {
        if(a2>a1) factor *= -1.;
        incr_ = factor*((2*M_PI - abs_delta)/(m_nsteps+1));
    }else{
        incr_ = interporlate_scalar(a1, a2);
    }
    values.resize(m_nsteps);
    double curr_v = a1 + incr_ ;
    for(int i=0; i < m_nsteps; i++)
    {
        values[i] = (curr_v);
        curr_v += incr_;
    } 
}

void CInterporlation::interporlate_scalar(double t1, double t2, vector<double>& values)
{
    double delta_scalar = interporlate_scalar(t1, t2);
    if(DEBUG) 
    {
      //  cout<<std::fixed<<"interporlate_scalar:" << delta_scalar<<" t1: "<< t1 <<" t2: "<<t2<<endl;
    }
    values.resize(m_nsteps);
    double curr_v = t1 + delta_scalar ;
    for(int i=0; i < m_nsteps; i++)
    {
        values[i] = (curr_v);
        curr_v += delta_scalar;
    }
}


double CInterporlation::interporlate_scalar(double t1, double t2)
{
    double delta_t = t2 - t1;
    return (delta_t/(double)(m_nsteps+1));
}


void CInterporlation::interporlate_pose(vector<double>& p1, vector<double>& p2, vector<vector<double> >& delta_p)
{
    tf::Quaternion q1(p1[3], p1[4], p1[5], p1[6]);
    tf::Quaternion q2(p2[3], p2[4], p2[5], p2[6]);

    double yaw1 = tf::getYaw(q1);
    double yaw2 = tf::getYaw(q2);

    if(DEBUG)
    {
        /*double roll1 ;// = tf::getRoll(q1);
        double roll2 ; // = tf::getRoll(q2);
        double pitch1 ; // = tf::getPitch(q1);
        double pitch2 ; // = tf::getPitch(q2);
        tf::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
        tf::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);
        // cout<<"yaw1: "<<yaw1<<" roll1: "<<roll1<<" pitch1: "<<pitch1<<endl;
        // cout<<"yaw2: "<<yaw2<<" roll2: "<<roll2<<" pitch2: "<<pitch2<<endl;
        cout<<"quaternion: "<<q1.x()<<" "<<q1.y()<<" "<<q1.z()<<" "<<q1.w()<<endl;
        tf::Quaternion qq1;
        qq1.setEuler( roll1, pitch1, yaw1 );
        cout<<"quaternion: "<<qq1.x()<<" "<<qq1.y()<<" "<<qq1.z()<<" "<<qq1.w()<<endl;

        // tf::Matrix3x3(qq1).getRPY(roll1, pitch1, yaw1);
        // cout<<"yaw1: "<<yaw1<<" roll1: "<<roll1<<" pitch1: "<<pitch1<<endl;*/
    }
    vector<double> delta_x; 
    vector<double> delta_y; 
    vector<double> delta_theta;

    interporlate_scalar(p1[0], p2[0], delta_x);
    interporlate_scalar(p1[1], p2[1], delta_y);
    // interporlate_scalar(yaw1, yaw2, delta_theta);
    interporlate_angle(yaw1, yaw2, delta_theta);
    if(DEBUG)
    {
        printV(delta_x, "delta_x");
        printV(delta_y, "delta_y");
        printV(delta_theta, "delta_theta");
    }

    vector<double> pose;
    pose.resize(7);
    delta_p.resize(m_nsteps, vector<double>() );
    double roll, pitch, yaw;
    roll = pitch = yaw = 0;
    for(int i=0; i<m_nsteps; i++)
    {
        pose[0] = delta_x[i];
        pose[1] = delta_y[i];
        pose[2] = 0;
        yaw = delta_theta[i];
        tf::Quaternion q;
        // q.setEuler(yaw, pitch, roll);
        q.setEuler(roll, pitch, yaw); // something misunderstanding!
        pose[3] = q.x();
        pose[4] = q.y();
        pose[5] = q.z();
        pose[6] = q.w();
        // delta_p.push_back(pose);
        delta_p[i] = pose;
    }
}

int CInterporlation::angle_map(double angle, double angle_stage)
{
    if(angle >= M_PI) 
        angle = 2*M_PI - angle;
    return static_cast<int>(angle/angle_stage) ;
}

void CInterporlation::statistic_angle_dif(vector<vector<double> >& poses)
{
    if(poses.size() <= 2) return ;
    vector<double>& p = poses[0];
    tf::Quaternion q(p[3], p[4], p[5], p[6]);
    double yaw1 = tf::getYaw(q);
    double yaw2; 
    map<int, int> statis;
    
    
    ofstream debug("debug.txt");

    for(int i=1; i<poses.size(); i++)
    {
        vector<double>& tp = poses[i];
        tf::Quaternion tq(tp[3], tp[4], tp[5], tp[6]);
        yaw2 = tf::getYaw(tq);
        int index = angle_map(fabs(yaw2 - yaw1)); 
        if(skip_interpolate(poses[i-1], poses[i]))
        {
            debug<<i-1<<"\t"<<i<<"\t"<<yaw1<<"\t"<<yaw2<<"\t"<<R2D(fabs(yaw2 - yaw1))<<endl;
        }
        map<int, int>::iterator it = statis.find(index);
        if(it == statis.end())
            statis[index] = 1;
        else
           it->second++;
        yaw1 = yaw2;
    }
    ofstream ouf("angle_statis.txt");
    map<int, int>::iterator it = statis.begin();
    while(it!=statis.end())
    {
        ouf<<it->first*ANGLE_VALUE<<"\t"<<(it->first+1)*ANGLE_VALUE<<"\t"<<it->second<<endl;
        ++it;
    }
}

void test_statistic_angle_dif(const char* fname, unsigned int steps)
{
    CInterporlation into(steps);
    ifstream inf(fname);
    if(!inf.is_open())
    {
        cout<<"failed to open file: "<<fname<<endl;
        return ;
    }
    vector< vector<double> > poses; 
    vector<double> t1; 
    vector<double> t2;
    into.getAllData(inf, poses, t1, t2);
    into.statistic_angle_dif(poses);
    cout<<"finish test_statistic_angle_dif()"<<endl;
}

