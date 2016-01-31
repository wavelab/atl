#include "awesomo/quadrotor.hpp"


Quadrotor::Quadrotor(void)
{
    this->subscribeToIMU();
}

void Quadrotor::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // TODO MUST CHECK XYZ
    this->roll = msg->orientation.x;
    this->pitch = msg->orientation.y;
    this->yaw = msg->orientation.z;
    this->omega = msg->orientation.w;

    ROS_INFO(
        "GOT: [%f, %f, %f, %f]",
        this->roll,
        this->pitch,
        this->yaw,
        this->omega
    );
}

void Quadrotor::subscribeToIMU(void)
{
    this->imu_orientation_sub = this->imu_orientation.subscribe(
        "/mavros/imu/data",
        1000,
        &Quadrotor::imuCallback,
        this
    );
}

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    Quadrotor quad;
    ros::spin();

    return 0;
}
