#include "awesomo/quadrotor.hpp"


Quadrotor::Quadrotor(void)
{
    // wait till connected to FCU
    this->waitForConnection();

    // subscribe to topics
    this->subscribeToMocap();
    // this->subscribeToIMU();

    // initialize clients to services
    this->mode_client = this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
    this->arming_client = this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);

    // initialize publishers
    this->pose_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 10);
    this->throttle_publisher = this->node.advertise<std_msgs::Float64>(THROTTLE_TOPIC, 10);
}

void Quadrotor::mocapCallback(const geometry_msgs::PoseStamped &msg)
{
    // mocap position
    this->mocap_x = msg.pose.position.x;
    this->mocap_y = msg.pose.position.y;
    this->mocap_z = msg.pose.position.z;

    // mocap orientation
    quat2euler(msg.pose.orientation, &this->mocap_roll, &this->mocap_pitch, &this->mocap_yaw);

    // print
    ROS_INFO(
        "GOT MOCAP: [%f, %f, %f]",
        rad2deg(this->mocap_roll),
        rad2deg(this->mocap_pitch),
        rad2deg(this->mocap_yaw)
    );
}

void Quadrotor::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    quat2euler(msg->orientation, &this->roll, &this->pitch, &this->yaw);

    ROS_INFO(
        "GOT MOCAP: [%f, %f, %f]",
        rad2deg(this->roll),
        rad2deg(this->pitch),
        rad2deg(this->yaw)
    );
}

void Quadrotor::subscribeToMocap(void)
{
    ROS_INFO("subcribing to [MOCAP]");
    this->mocap_subscriber = this->node.subscribe(
        MOCAP_TOPIC,
        100,
        &Quadrotor::mocapCallback,
        this
    );
}

void Quadrotor::subscribeToIMU(void)
{
    ROS_INFO("subcribing to [IMU_DATA]");
    this->imu_subscriber = this->node.subscribe(
        IMU_TOPIC,
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
    ROS_INFO("waiting for FCU ...");

    while (ros::ok() && this->state.connected) {
        ros::spinOnce();
        sleep(1);
    }
}

int Quadrotor::arm(void)
{
    mavros_msgs::CommandBool arm_req;

    // setup
    ROS_INFO("arming awesomo ...");
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
    ROS_INFO("disarming awesomo ...");
    arm_req.request.value = false;

    // arm
    if (this->arming_client.call(arm_req)) {
        ROS_INFO("awesomo disarmed!");
    } else {
        ROS_ERROR("failed to disarm awesomo!");
    }

    return 0;
}

int Quadrotor::setOffboardModeOn(void)
{
    mavros_msgs::SetMode mode;

    // setup
    mode.request.custom_mode = "OFFBOARD";

    if (mode_client.call(mode) && mode.response.success) {
        ROS_INFO("Offboard enabled");
        return 0;
    } else {
        return -1;
    }
}

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
	ros::NodeHandle nh;

    ros::Rate rate(10.0);  // publishing rate MUST be faster than 2Hz
    Quadrotor quad;

	std_msgs::Float64 cmd_thr;
	cmd_thr.data = 0.4;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

	ROS_INFO("running ...");
    while (ros::ok()){
        // create quaternion from roll pitch yaw
	    tf::Quaternion quat = euler2quat(deg2rad(10), quad.pitch, quad.yaw);

	    // sending orientation in quaternions
	    pose.pose.orientation.x = quat.x();
	    pose.pose.orientation.y = quat.y();
	    pose.pose.orientation.z = quat.z();
	    pose.pose.orientation.w = quat.w();

        // publish pose and throttle
        quad.pose_publisher.publish(pose);
        quad.throttle_publisher.publish(cmd_thr);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
