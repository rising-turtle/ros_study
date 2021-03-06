#include <map>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "point.h"

#define SQ(x) ((x)*(x))

#define DEBUG 1

using namespace std;

void rel_dis(const char* file1, const char* file2);
bool getV(const char* f, map<double, vector<double> >& );
double angleDis(double a1, double a2);
void fromabs2rel(const map<double, vector<double> >& , map<double, vector<double> >&);

int main(int argc, char* argv[])
{
    if(argc < 3 )
    {
        cout<<"usage: abs_dis $file1 $file2"<<endl;
        return -1;
    }
    rel_dis(argv[1], argv[2]);
    return 0; 
}

void abs_dis(const char* f1, const char* f2)
{
    map<double, vector<double> > abs_v1; 
    map<double, vector<double> > abs_v2;
    if(!getV(f1, abs_v1))
    {
        cout<<"not exist: "<<f1<<endl; 
        return ;
    }
    if(!getV(f2, abs_v2))
    {
        cout<<"not exist: "<<f2<<endl;
        return ;
    }
    cout<<"succeed to read data!"; 
    map<double, vector<double> > v1; 
    map<double, vector<double> > v2;
    fromabs2rel(abs_v1, v1); 
    fromabs2rel(abs_v2, v2);
    
    // calculate mean value; 
    int num = 0;
    double dx, dy, dis, dtheta, tmpx, tmpy; 
    double meanDx, meanDy, meanDis, meanDtheta; 
    dx = dy = dis = dtheta = 0 ;
    if(v2.size() > v1.size()) v1.swap(v2); 
    map<double, vector<double> >::iterator it_query = v2.begin();
    map<double, vector<double> >::iterator it_target ;
    vector<double> key;
    while(it_query != v2.end())
    {
        it_target = v1.find(it_query->first); 
        if(it_target != v1.end())
        {
            num++;
            key.push_back(it_query->first);

            vector<double>& qv1 = it_target->second; 
            vector<double>& qv2 = it_query->second; 
            tmpx = fabs(qv1[0] - qv2[0]);
            tmpy = fabs(qv1[1] - qv2[1]);
            dx += tmpx;
            dy += tmpy; 
            dis += sqrt(SQ(tmpx) + SQ(tmpy)); 
            dtheta += angleDis(qv1[2], qv2[2]);
#ifdef DEBUG
            {
                static ofstream d_ouf("debug.txt"); 
                d_ouf<<fabs(qv1[0]-qv2[0])<<" "<<fabs(qv1[1] - qv2[1])<<" "<<angleDis(qv1[2], qv2[2])<<endl;
            }
#endif
        }else
        {
            cout<<"what? no value in traget for key: "<<it_query->first<<endl;
        }
        it_query ++; 
    }
    if(num == 0)
    {
        cout<<"what? no key matched !"<<endl;
        return ;
    }
    
    assert(num == key.size());

    meanDx = dx/(double)num; 
    meanDy = dy/(double)num;
    meanDis = dis/(double)num; 
    meanDtheta = dtheta/(double)num;
    
    cout<<"absolute mean values: "<<endl;
    cout<<"dx: \t"<<meanDx<<endl;
    cout<<"dy: \t"<<meanDy<<endl;
    cout<<"dis: \t"<<meanDis<<endl;
    cout<<"dtheta: \t"<<meanDtheta<<endl;

    // calculate std value; 
    dx = dy = dis = dtheta = 0 ;
    double stdDx, stdDy, stdDis, stdDtheta; 
    for(int i=0; i<key.size(); i++)
    {
        vector<double>& qv1 = v1.find(key[i])->second; 
        vector<double>& qv2 = v2.find(key[i])->second;
        tmpx = fabs(qv1[0] - qv2[0]); 
        tmpy = fabs(qv1[1] - qv2[1]);
        dx += SQ( tmpx - meanDx); 
        dy += SQ( tmpy - meanDy); 
        dis += SQ( sqrt(SQ(tmpx) + SQ(tmpy)) - meanDis); 
        dtheta += SQ(angleDis(qv1[2], qv2[2]) - meanDtheta);
    }
    stdDx = sqrt(dx/(double)(num-1.)); 
    stdDy = sqrt(dy/(double)(num-1.)); 
    stdDis = sqrt(dis/(double)(num-1.)); 
    stdDtheta = sqrt(dtheta/(double)(num-1.));
    
    cout<<"absolute std valuse: "<<endl;
    cout<<"stdx: \t"<<stdDx<<endl;
    cout<<"stdy: \t"<<stdDy<<endl;
    cout<<"stdDis: \t"<<stdDis<<endl;
    cout<<"stdThta: \t"<<stdDtheta<<endl;
}

void fromabs2rel(const map<double, vector<double> >& abs, map<double, vector<double> >& rel)
{
    map<double, vector<double> >::const_iterator it_curr = abs.begin();
    map<double, vector<double> >::const_iterator it_prev = it_curr;
    rel.clear();
    it_curr++;
    vector<double> p(3);
    while(it_curr != abs.end())
    {
        OrientedPoint2D prev(it_prev->second[0], it_prev->second[1], it_prev->second[2]); 
        OrientedPoint2D curr(it_curr->second[0], it_curr->second[1], it_curr->second[2]);
        OrientedPoint2D trans = prev.ominus(curr); 
        p[0] = trans.x; p[1] = trans.y; p[2] = trans.theta; 
        rel.insert(make_pair(it_prev->first, p));
        it_prev = it_curr;
        it_curr++;
    }
    cout<<"finish transform abs_pose to rel_pose"<<endl;
}

bool getV(const char* f, map<double, vector<double> >& dist)
{
    ifstream inf(f);
    if(!inf.is_open())
    {
        cout<<"failed to open file: "<<f<<endl;
        return false;
    }
    char line[4096];
    vector<double> v(3); 
    double timestamp;
    string delim(" ,\t");
    while(inf.getline(line, sizeof(line)))
    {
        timestamp = atof(strtok(line, delim.c_str()));
        v[0] = atof(strtok(NULL, delim.c_str()));
        v[1] = atof(strtok(NULL, delim.c_str()));
        v[2] = atof(strtok(NULL, delim.c_str()));
        dist.insert(make_pair(timestamp, v));
    }
    return true;
}


double angleDis(double a1, double a2)
{
    double ret = fabs(a1 - a2);
    while(ret > M_PI)
    {
        ret = fabs(2*M_PI - ret);
    }
    return ret;
}



