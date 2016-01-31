#ifndef __CORE_H__
#define __CORE_H__

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


class Quadrotor
{
    private:
        ros::NodeHandle imu_orientation;
        ros::Subscriber imu_orientation_sub;

        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void subscribeToIMU(void);

    public:
        float roll;
        float pitch;
        float yaw;
        float omega;

        Quadrotor(void);
};

#endif
