#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>


#define IMU_DATA "/mavros/imu/data"

#define RC_SERVICE "/mavros/rc/override"
#define ARM_SERVICE "/mavros/cmd/arming"
#define SET_MODE_SERVICE "/mavros/set_mode"



class Quadrotor
{
    private:
        mavros_msgs::State state;
        ros::NodeHandle node_handle;
        ros::Subscriber imu_orientation_sub;

        ros::ServiceClient set_mode_client;
        ros::ServiceClient arming_client;
        ros::ServiceClient motor_client;

        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void subscribeToIMU(void);
        void stateCallback(const mavros_msgs::State::ConstPtr &msg);
        void waitForConnection(void);

    public:
        float roll;
        float pitch;
        float yaw;
        float omega;

        Quadrotor(void);
        int arm(void);
        int disarm(void);
};

#endif
