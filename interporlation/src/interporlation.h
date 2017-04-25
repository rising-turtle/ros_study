#ifndef INTERPORLATION_H
#define INTERPORLATION_H

#include <string>
#include <vector>
#include <fstream>
#include <cmath>
using namespace std;
#define STAGE_NUM 18
#define ANGLE_STAGE (M_PI/STAGE_NUM) // 10 degree
#define ANGLE_VALUE (180/STAGE_NUM) 

#define R2D(x) (((x)/M_PI)*180)
#define D2R(x) (((x)*M_PI)/180.)
#define SKIP_ANGLE (ANGLE_VALUE)

class CInterporlation
{
    friend void test_statistic_angle_dif(const char* fname, unsigned int steps);
public:
    CInterporlation(unsigned int steps = 5); 
    ~CInterporlation();
    bool interpolate(string input_f, string suffix="");
    
protected:
    void getAllData(ifstream& inf, vector< vector<double> >& pose, vector<double>& time_1, vector<double>& time_2);
    double interporlate_scalar(double t1, double t2);
    void interporlate_scalar(double t1, double t2, vector<double>&);
    void interporlate_angle(double angle1, double angle2, vector<double>&);
    void interporlate_pose(vector<double>& p1, vector<double>& p2, vector< vector<double> >& delta_p);
    void write_log(ofstream& , double t1, double t2, vector<double>& pose );
    void statistic_angle_dif(vector< vector<double> >& poses);
    int angle_map(double angle, double angle_stage = ANGLE_STAGE);
    bool skip_interpolate(vector<double>& p1, vector<double>& p2);
    unsigned int m_nsteps; 
};

extern void test_statistic_angle_dif(const char* fname, unsigned int steps);

#endif
