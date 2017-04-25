#include "ros/ros.h"
#include <tf/tf.h>
#include <cmath>
#include <iostream>

#define R2D(r) (((r)*180.)/M_PI)

using namespace std;

void printTF(tf::Transform t, const char* prefix)
{
    cout<<prefix<<endl;
    tf::Vector3 tr = t.getOrigin();
    double roll, pitch, yaw;  
    tf::Matrix3x3(t.getRotation()).getRPY(roll, pitch, yaw);

    cout<<fixed<<"x: "<<tr.getX()<<" y: "<<tr.getY()<<" z: "<<tr.getZ()<<endl; 
    cout<<"roll: "<<R2D(roll)<<" pitch: "<<R2D(pitch)<<" yaw: "<<R2D(yaw)<<endl;
}


void test();
void test3();

int main(int argc, char* argv[])
{
    test();
    cout<<"finished !"<<endl;
    return 0 ;
}

void test3()
{
    tf::Transform r1, r2, r3; 
    r1.setIdentity();
    r2.setIdentity();
    r3.setIdentity();
    // r1.setOrigin(tf::Vector3(1, 2, 3));
    r1.setRotation(tf::createQuaternionFromRPY( -M_PI/2., (M_PI)/4., (M_PI)/6.));
    // r1.setRotation(tf::createQuaternionFromRPY( 0, 0 , (M_PI)/2.));


    r2.setOrigin(tf::Vector3(0, 2, 1));
    r2.setRotation(tf::createQuaternionFromRPY(0, 0, (M_PI)/6.));

    printTF(r2, "before multiple"); 
    r3 = r1*r2*r1.inverse();
    printTF(r3, "after multiple");

    tf::Transform R(r1.getBasis(), tf::Vector3(0,0,0));
    tf::Transform R0(r2.getBasis(), tf::Vector3(0,0,0));
    tf::Vector3 t0 = r2.getOrigin();
    tf::Vector3 t = r1.getOrigin();

    tf::Vector3 t1 = (R*R0*R.inverse()*t)*(-1) + R*t0 + t; 
    // cout<<"t1: "<<t1.x()<<" "<<t1.y()<<" "<<t1.z()<<endl;
    tf::Transform R1 = R*R0*R.inverse();
    R1.setOrigin(t1);
    printTF(R1, "equation = ");
}

void test()
{
    tf::Transform odo2map, odo_rel, map_first, odo_mea; 
    odo2map.setIdentity();
    odo_rel.setIdentity();
    map_first.setIdentity();
    odo_mea.setIdentity();
    odo_rel.setIdentity();
    odo2map.setOrigin(tf::Vector3(100,0,0));
    // odo2map.setRotation(tf::createQuaternionFromRPY(0 , 0, (M_PI)/6.));
    
    // odoTrans_first.setOrigin(tf::Vector3(1, 2, 0)); 
    
    odo_mea.setIdentity();
    odo_mea.setOrigin(tf::Vector3(3, 4, 0));
    odo_mea.setRotation(tf::createQuaternionFromRPY(0, 0, (M_PI)/6.));

    // odo_mea = odoTrans_first.inverse()*odo_mea; 
    printTF(odo_mea, "odo_mea");
    odo_rel=odo2map*(odo_mea)*odo2map.inverse();
    printTF(odo_rel, "odo_rel");
    
    tf::Transform rel2map; 
    rel2map = map_first*odo_rel; 
    printTF(rel2map, "rel2map");

}


void test2()
{
    tf::Transform world, world2f1, world2f2, f12p, f22p; 
    world.setIdentity();
    world2f1.setIdentity();
    world2f1.setOrigin(tf::Vector3(0,2,0)); 
    world2f1.setRotation(tf::createQuaternionFromRPY(0, 0, M_PI/3));

    printTF(world2f1, "world2f1");
    printTF(world2f1.inverse(), "world2f1.inverse()");

    printTF(world2f1*world2f1.inverse(), "multiple");
}


