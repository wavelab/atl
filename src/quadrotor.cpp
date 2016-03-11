#include "awesomo/quadrotor.hpp"


static inline double deg2rad(double d)
{
    return d * (M_PI / 180);
}

static inline double rad2deg(double d)
{
    return d * (180 / M_PI);
}

Quadrotor::Quadrotor(void)
{
    // wait till connected to FCU
    this->waitForConnection();

    // subscribe to topics
    this->subscribeToIMU();

    // initialize clients to services
    this->mode_client = this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
    this->arming_client = this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);

    // initialize publishers
    this->pose_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 10);
    this->throttle_publisher = this->node.advertise<std_msgs::Float64>(THROTTLE_TOPIC, 10);
}

void Quadrotor::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double x;
    double y;
    double z;
    double w;

    x = msg->orientation.x;
    y = msg->orientation.y;
    z = msg->orientation.z;
    w = msg->orientation.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(this->roll, this->pitch, this->yaw);

    ROS_INFO(
        "GOT: [%f, %f, %f]",
        rad2deg(this->roll),
        rad2deg(this->pitch),
        rad2deg(this->yaw)
    );
}

void Quadrotor::subscribeToIMU(void)
{
    ROS_INFO("subcribing to [IMU_DATA]");
    this->imu_orientation_sub = this->node.subscribe(
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
	    tf::Quaternion quat = tf::createQuaternionFromRPY(
            deg2rad(10),
            quad.pitch,
            quad.yaw
	    );

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
