#ifndef __QUADROTOR_H__
#define __QUADROTOR_H__

#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>


#define ARM_SERVICE "/mavros/cmd/arming"
#define SET_MODE_SERVICE "/mavros/set_mode"
#define IMU_DATA "/mavros/imu/data"


class Quadrotor
{
    private:
        mavros_msgs::State state;
        ros::NodeHandle node_handle;
        ros::Subscriber imu_orientation_sub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;

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
