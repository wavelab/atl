#include "awesomo/quadrotor.hpp"


Quadrotor::Quadrotor(void)
{
    // publish rate
    ros::Rate rate(20.0);

    // wait till connected to FCU
    this->waitForConnection();

    // subscribe to topics
    this->subscribeToIMU();

    // initialize clients to services
    this->set_mode_client = this->node_handle.serviceClient<mavros_msgs::SetMode>(SET_MODE_SERVICE);
    this->arming_client = this->node_handle.serviceClient<mavros_msgs::CommandBool>(ARM_SERVICE);
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
    ROS_INFO("subcribing to [IMU_DATA]");
    this->imu_orientation_sub = this->node_handle.subscribe(
        IMU_DATA,
        1000,
        &Quadrotor::imuCallback,
        this
    );
}

void Quadrotor::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    state = *msg;
}

void Quadrotor::waitForConnection(void)
{
    while (ros::ok() && this->state.connected) {
        ros::spinOnce();
        sleep(1);
    }
}

int Quadrotor::arm(void)
{
    mavros_msgs::CommandBool arm_req;

    // setup
    ROS_INFO("arming awesomo!");
    arm_req.request.value = true;

    // arm
    if (this->arming_client.call(arm_req)) {
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
    if (this->arming_client.call(arm_req)) {
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

    // run quadcopter
    Quadrotor quad;
    quad.arm();
    sleep(2);
    quad.disarm();

    // loop
    ros::spin();

    return 0;
}
