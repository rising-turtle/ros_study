#include <ros/ros.h>
#include <tf/transform_listener.h>
// #include <turtlesim/Velocity.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

void printTF(tf::StampedTransform& tf);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;

    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle =
        node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    add_turtle.call(srv);

    ros::Publisher turtle_vel =
        node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    tf::TransformListener listener;
    tf::TransformListener listener1; 
    tf::TransformListener listener2; 

    ros::Rate rate(10.0);
    while (node.ok())
    {
        tf::StampedTransform transform;
        tf::StampedTransform world2t1;
        tf::StampedTransform world2t2;
        try{
            
            listener1.lookupTransform("/world", "turtle1", ros::Time(0), world2t1);
            listener2.lookupTransform("/world", "turtle2", ros::Time(0), world2t2);
            listener.lookupTransform("/turtle2", "/turtle1",
                    ros::Time(0), transform);
            printTF(world2t1); 
            printTF(world2t2);
            printTF(transform);
            tf::StampedTransform tf_t(world2t2.inverse()*world2t1, ros::Time(0), "/turtle2", "/turtle1"); 
            printTF(tf_t);
            // ROS_INFO("/turtle2 to /turtle1: x: %lf, y: %lf, z: %lf;  x: %lf y: %lf z: %lf w: %lf", transform.getOrigin().x(), \
                        transform.getOrigin().y(), transform.getOrigin().z(), transform.getRotation().x(), \
                        transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        // turtlesim::Velocity vel_msg;
        geometry_msgs::Twist vel_msg;
        double dis = sqrt(pow(transform.getOrigin().x(), 2) +
                pow(transform.getOrigin().y(), 2)); 
        double angle = atan2(transform.getOrigin().y(),
                 transform.getOrigin().x()); 
        vel_msg.angular.z = 4 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
               // vel_msg.linear.x = dis * cos(angle);   
        vel_msg.linear.y =  dis * sin(angle); 
        // ROS_WARN("velocity: linear.x = %lf, linear.y = %lf, angular.z = %lf", vel_msg.linear.x , vel_msg.linear.y, vel_msg.angular.z );
        turtle_vel.publish(vel_msg);

        rate.sleep();
    }
    return 0;
};

void printTF(tf::StampedTransform& transform)
{
    ROS_INFO("%s to %s: x: %lf, y: %lf, z: %lf;  x: %lf y: %lf z: %lf w: %lf", transform.frame_id_.c_str(), \
            transform.child_frame_id_.c_str(), transform.getOrigin().x(), \
            transform.getOrigin().y(), transform.getOrigin().z(), transform.getRotation().x(), \
            transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
}

