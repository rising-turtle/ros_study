#include <iostream>
#include <string>
#include <stdlib.h>
#include "interporlation.h"

using namespace std;

int main(int argc, char* argv[])
{
    string fname("path_odo.txt"); 
    string outf("");
    int m_inporlate_num = 5;
    if(argc >=3)
    {
        fname = string(argv[1]);
        m_inporlate_num = atoi(argv[2]);
        
        if(argc > 3)
            outf = argv[3];
    }else{
        cout<<"xxx inputf interporlate_num [outf]"<<endl;
    }
    // test_statistic_angle_dif(fname.c_str(), m_inporlate_num);
    
    CInterporlation intpo(m_inporlate_num);
    if(intpo.interpolate(fname, outf))
    {
        cout<<"succeed to interporlate "<<fname<<endl;
    }else{
        cout<<"failed to interporlate "<<fname<<endl;
    }
    return 0;
}
