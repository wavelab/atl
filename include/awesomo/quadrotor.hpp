#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>
#include <cmath>

#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include "awesomo/util.hpp"


// CONSTANTS
#define IMU_TOPIC "/mavros/imu/data"
#define ARM_TOPIC "/mavros/cmd/arming"
#define MOCAP_TOPIC "/awesomo/mocap/pose"
#define MODE_TOPIC "/mavros/set_mode"
#define POSE_TOPIC "/mavros/setpoint_attitude/attitude"
#define THROTTLE_TOPIC "/mavros/setpoint_attitude/att_throttle"




class Quadrotor
{
    private:
        mavros_msgs::State state;
        ros::NodeHandle node;
        ros::Subscriber mocap_subscriber;
        ros::Subscriber imu_subscriber;

        ros::ServiceClient mode_client;
        ros::ServiceClient arming_client;

        void mocapCallback(const geometry_msgs::PoseStamped &msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void subscribeToMocap(void);
        void subscribeToIMU(void);
        void stateCallback(const mavros_msgs::State::ConstPtr &msg);
        void waitForConnection(void);

    public:
        double roll;
        double pitch;
        double yaw;

        double mocap_x;
        double mocap_y;
        double mocap_z;

        double mocap_roll;
        double mocap_pitch;
        double mocap_yaw;

        ros::Publisher throttle_publisher;
        ros::Publisher pose_publisher;

        Quadrotor(void);
        int arm(void);
        int disarm(void);
        int setOffboardModeOn(void);
};

#endif
