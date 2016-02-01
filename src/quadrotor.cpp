#include "awesomo/quadrotor.hpp"


Quadrotor::Quadrotor(void)
{
    ros::NodeHandle nh;

    this->subscribeToIMU();
    this->arm_client = nh.serviceClient<mavros_msgs::CommandBool>(ARM_SERVICE);
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
    ros::NodeHandle nh;

    ROS_INFO("subcribing to [IMU_DATA]");
    this->imu_orientation_sub = nh.subscribe(
        IMU_DATA,
        1000,
        &Quadrotor::imuCallback,
        this
    );
}

int Quadrotor::arm(void)
{
    mavros_msgs::CommandBool arm_req;

    // setup
    ROS_INFO("arming awesomo!");
    arm_req.request.value = true;

    // arm
    if (this->arm_client.call(arm_req)) {
        ROS_INFO("awesomo armed!");
    } else {
        ROS_ERROR("failed to arm awesomo!");
    }

    return 0;
}

int Quadrotor::disarm(void)
{
    mavros_msgs::CommandBool arm_req;

    // setup
    ROS_INFO("disarming awesomo!");
    arm_req.request.value = false;

    // arm
    if (this->arm_client.call(arm_req)) {
        ROS_INFO("awesomo disarmed!");
    } else {
        ROS_ERROR("failed to disarm awesomo!");
    }

    return 0;
}

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    Quadrotor quad;
    quad.arm();
    sleep(2);
    quad.disarm();
    ros::spin();

    return 0;
}
