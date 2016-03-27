#include "awesomo/ros/quadrotor_node.hpp"


static int fltcmp(double f1, double f2)
{
	if (fabs(f1 - f2) <= 0.0001) {
		return 0;
	} else if (f1 > f2) {
		return 1;
	} else {
		return -1;
	}
}

Quadrotor::Quadrotor(void)
{
    // wait till connected to FCU
    this->waitForConnection();

    // subscribe to topics
    // this->subscribeToPose();
    // this->subscribeToMocap();
    // this->subscribeToIMU();

    // initialize clients to services
    this->mode_client = this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
    this->arming_client = this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);

    // initialize publishers
    this->position_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_TOPIC, 50);
    // this->attitude_publisher = this->node.advertise<geometry_msgs::PoseStamped>(ATTITUDE_TOPIC, 50);
    // this->throttle_publisher = this->node.advertise<std_msgs::Float64>(THROTTLE_TOPIC, 50);
}

void Quadrotor::poseCallback(const geometry_msgs::PoseStamped &msg)
{
    //  mocapposition
    this->pose_x = msg.pose.position.x;
    this->pose_y = msg.pose.position.y;
    this->pose_z = msg.pose.position.z;

    //  mocaporientation
    quat2euler(msg.pose.orientation, &this->pose_roll, &this->pose_pitch, &this->pose_yaw);

    // print
    ROS_INFO(
        "GOT POSE: [roll: %f, pitch: %f, yaw: %f, x: %f, y: %f, z: %f]",
        rad2deg(this->pose_roll),
        rad2deg(this->pose_pitch),
        rad2deg(this->pose_yaw),
        pose_x,
        pose_y,
        pose_z
    );
}

void Quadrotor::subscribeToPose(void)
{
    ROS_INFO("subcribing to [POSE]");
    this->pose_subscriber = this->node.subscribe(
        POSE_TOPIC,
        50,
        &Quadrotor::poseCallback,
        this
    );
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

void Quadrotor::subscribeToMocap(void)
{
    ROS_INFO("subcribing to [MOCAP]");
    this->mocap_subscriber = this->node.subscribe(
        MOCAP_TOPIC,
        50,
        &Quadrotor::mocapCallback,
        this
    );
}

void Quadrotor::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    quat2euler(msg->orientation, &this->roll, &this->pitch, &this->yaw);

    ROS_INFO(
        "GOT IMU: [%f, %f, %f]",
        rad2deg(this->roll),
        rad2deg(this->pitch),
        rad2deg(this->yaw)
    );
}

void Quadrotor::subscribeToIMU(void)
{
    ROS_INFO("subcribing to [IMU_DATA]");
    this->imu_subscriber = this->node.subscribe(
        IMU_TOPIC,
        50,
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
    ros::Time last_request;

    ros::Rate rate(50.0);  // publishing rate MUST be faster than 2Hz
    Quadrotor quad;
	geometry_msgs::PoseStamped pose;

    ROS_INFO("running ...");
    // quad.arm();
    // quad.setOffboardModeOn();
	last_request = ros::Time::now();
	int count = 1;
	int index = 0;

    while (ros::ok()){
        pose.header.stamp = ros::Time::now();
        pose.header.seq = count;
        pose.header.frame_id = 1;
        ROS_INFO("x: 0.0\ty: 0.0\tz: 1.0");
		pose.pose.position.z = 1.0;
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
		count++;

		// if (ros::Time::now() - last_request > ros::Duration(10.0)) {
        //     if (index == 0) {
        //         pose.pose.position.x = 0.5;
        //         pose.pose.position.y = 0.5;
        //         index++;
        //         ROS_INFO("x: 0.5\ty: 0.5");
        //     } else if (index == 1) {
        //         pose.pose.position.x = 0.5;
        //         pose.pose.position.y = -0.5;
        //         ROS_INFO("x: 0.5\ty: -0.5");
        //         index++;
        //     } else if (index == 2) {
        //         pose.pose.position.x = -0.5;
        //         pose.pose.position.y = -0.5;
        //         ROS_INFO("x: -0.5\ty: -0.5");
        //         index++;
        //     } else if (index == 3) {
        //         pose.pose.position.x = -0.5;
        //         pose.pose.position.y = 0.5;
        //         ROS_INFO("x: -0.5\ty: 0.5");
        //         index = 0;
        //     }
        //
		// 	last_request = ros::Time::now();
		// }

		// publish
		quad.position_publisher.publish(pose);

		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
