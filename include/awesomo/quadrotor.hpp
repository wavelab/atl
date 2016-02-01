#ifndef __QUADROTOR_H__
#define __QUADROTOR_H__

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/CommandBool.h>


#define ARM_SERVICE "/mavros/cmd/arming"
#define IMU_DATA "/mavros/imu/data"


class Quadrotor
{
    private:
        ros::Subscriber imu_orientation_sub;
        ros::ServiceClient arm_client;

        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void subscribeToIMU(void);

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
