//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: Assylbek Dakibay
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"awesomo_pub_setpoints");
    ros::NodeHandle n;
    ros::Publisher awesomo_set_point = n.advertise<geometry_msgs::PoseStamped>(
                "/mavros/setpoint_position/local", 100);
    ros::Rate loop_rate(100);
    ros::spinOnce();

    geometry_msgs::PoseStamped msg;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;


    int count;
    while(ros::ok()){
        msg.header.stamp  = ros::Time::now();
        msg.header.seq = count;
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
    return 0;
}
