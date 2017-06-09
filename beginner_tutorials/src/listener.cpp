#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

bool g_exit = false; 

void exitCallback(const std_msgs::BoolPtr& e)
{
  g_exit = true;
  ROS_INFO("listener.cpp: let' exit! ");
}

bool g_syn_flag = false;
void synCallback(const std_msgs::BoolPtr& b)
{
  g_syn_flag = b->data;
  ROS_INFO("listener.cpp: listen to the syn!");
}

bool g_get_msg = false;
int g_count = 0;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  g_get_msg = true;
  ++g_count; 
  // std_msgs::IntPtr num(new std_msgs::Int);
}

void sr_imgCB(const sensor_msgs::ImageConstPtr& img_msg)
{
  ROS_INFO("I receive img %d", ++g_count); 
  // display it? ok, let' do it 
  cv_bridge::CvImagePtr cv_ptr; 
  cv_ptr = cv_bridge::toCvCopy(img_msg); 
  cv::Mat cv_img = cv_ptr->image ; 
  cv::imshow("receive and show, ", cv_img);
  cv::waitKey(20); 
  // ROS_INFO("listener.cpp: now g_count = %d", g_count);
  g_get_msg = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber syn_sub_ = n.subscribe("/syn", 1, synCallback);
  ros::Subscriber exit_sub_ = n.subscribe("/exit", 1, exitCallback);
  ros::Publisher  num_pub_ = n.advertise<std_msgs::Int32>("/num_rece", 1); 
  ros::Publisher  ack_pub_ = n.advertise<std_msgs::Bool>("/ack", 1);
  
  // subscribe sw_image 
  ros::Subscriber img_sub_ = n.subscribe<sensor_msgs::Image>("/camera/sr_image", 1, sr_imgCB);

  ros::Rate r(50); 
  ROS_INFO("listener.cpp: wait for talker to send syn!");
  while(n.ok() && !g_syn_flag)
  {
    ros::spinOnce();
    r.sleep();
  }
  std_msgs::BoolPtr ack_f(new std_msgs::Bool); 
  ack_f->data = true; 
  ack_pub_.publish(ack_f);
  ROS_INFO("listener.cpp: after syn ack!");
  while(n.ok() && !g_exit)
  {
    if(g_get_msg)
    {
      // get msg 
      std_msgs::Int32Ptr num(new std_msgs::Int32); 
      num->data = g_count;
      num_pub_.publish(num);
      g_get_msg = false;
    }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
