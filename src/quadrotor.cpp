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
    this->set_mode_client = this->node_handle.serviceClient<mavros_msgs::SetMode>(SET_MODE_SERVICE);
    this->arming_client = this->node_handle.serviceClient<mavros_msgs::CommandBool>(ARM_SERVICE);
    // this->motor_client = this->node_handle.serviceClient<mavros_msgs::RCIn>(RC_SERVICE);
}

void Quadrotor::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double x;
    double y;
    double z;
    double w;

    // TODO MUST CHECK XYZ
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


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    // ros::Rate rate(10.0);  // publish rate
	ros::NodeHandle nh;

    // run quadcopter
    Quadrotor quad;
	ROS_INFO("running...");

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);
	ros::Publisher throttle_pub = nh.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
	ROS_INFO("waiting for FCU connection...");
    while (ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);


    // std::cout << quat.x() << quat.y() << quat.z() << quat.w() << std::endl;

	std_msgs::Float64 cmd_thr;
	cmd_thr.data = 0.4;

	// ROS_INFO("sending setpoints ...");
    // // send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";
    // if (set_mode_client.call(offb_set_mode) &&
    //     offb_set_mode.response.success){
    //     ROS_INFO("Offboard enabled");
    // }

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;
    //
    // ros::Time last_request = ros::Time::now();
    //
    // // if (current_state.mode != "OFFBOARD" &&
    // //     (ros::Time::now() - last_request > ros::Duration(10.0))){
    // //     if( set_mode_client.call(offb_set_mode) &&
    // //         offb_set_mode.response.success){
    // //         ROS_INFO("Offboard enabled");
    // //     }
    // //     last_request = ros::Time::now();
    // // } else {
    // //     if( !current_state.armed &&
    // //         (ros::Time::now() - last_request > ros::Duration(10.0))){
    // //         if( arming_client.call(arm_cmd) &&
    // //             arm_cmd.response.success){
    // //             ROS_INFO("Vehicle armed");
    // //         }
    // //         last_request = ros::Time::now();
    // //     }
    // // }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

	ROS_INFO("looping ...");
    while(ros::ok()){
		ROS_INFO("publishing pose and throttle");

	    tf::Quaternion quat = tf::createQuaternionFromRPY(
            deg2rad(10),
            quad.pitch,
            quad.yaw
	    );

	    // send a few setpoints before starting
	    pose.pose.orientation.x = quat.x();
	    pose.pose.orientation.y = quat.y();
	    pose.pose.orientation.z = quat.z();
	    pose.pose.orientation.w = quat.w();

        local_pos_pub.publish(pose);
        throttle_pub.publish(cmd_thr);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
