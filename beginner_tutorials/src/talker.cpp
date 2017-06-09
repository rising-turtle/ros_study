#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "SR_reader.h"
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

bool g_ack_syn = false; 
void ackCallback(const std_msgs::BoolPtr& ack)
{
  g_ack_syn = ack->data;
  cout<<"talker.cpp: get ack!"<<endl;
}

int g_rece_num = 0;
void numCallback(const std_msgs::Int32Ptr& num)
{
  g_rece_num = num->data;
}

cv::Mat from_SR_to_mat(sr_data& d)
{
    unsigned char* p =  (unsigned char*)(&d.intensity_[0]);
    cv::Mat i_img(sr_data::_sr_height, sr_data::_sr_width, CV_16UC1, p, 
        sr_data::_sr_width*sizeof(sr_data::_sr_type));
    return i_img;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher syn_pub_ = n.advertise<std_msgs::Bool>("/syn", 1);
  ros::Publisher exit_pub_ = n.advertise<std_msgs::Bool>("/exit", 1);
  ros::Subscriber ack_sub_ = n.subscribe("/ack", 1, ackCallback); 
  ros::Subscriber num_sub_ = n.subscribe("/num_rece", 1, numCallback);
  
  // publish sw_image 
  ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("/camera/sr_image", 1);

  ros::Rate loop_rate(10);
  std_msgs::BoolPtr b_syn_ok(new std_msgs::Bool);
  b_syn_ok->data = true;
  
  // load SR data first 
  CSReader r; 
  if(!r.loadAllData())
  {
    ROS_ERROR("talker.cpp: no SR data is loaded!");
    return -1;
  }
  
    ROS_INFO("talker.cpp: wait for syn ack!");
    // syn first 
    while(ros::ok() && !g_ack_syn)
    {
      syn_pub_.publish(b_syn_ok);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("talker.cpp: after syn ack!");
       
    // while(ros::ok() && g_rece_num < 20)
    // send the SR_image 
    int count = 0; // used to get ack that the image has been received
    ros::Time msg_timestamp; // ros time, used in the msg.header
    CSReader::iterator it = r.begin();
    cv_bridge::CvImage out_msg; 
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // image type

    while(ros::ok() && it != r.end())
    {
      
      /*  // then send the string
      std_msgs::String msg;
      std::stringstream ss;
      ss << "send number " << count;
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
      chatter_pub.publish(msg);
      */
      
      sr_data d = *it; 
      cv::Mat cv_img = from_SR_to_mat(d); 
      msg_timestamp = ros::Time();
      out_msg.header.stamp = msg_timestamp;
      out_msg.header.seq = count+1;
      out_msg.image = cv_img;
      img_pub.publish(out_msg);

      while(g_rece_num != count && ros::ok())
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
      ROS_INFO("talker.cpp: number of communicate data: %d , while count = %d", g_rece_num, count);
      ++count;
      ++it;
    }
  exit_pub_.publish(b_syn_ok);
  return 0;
}
