#include "awesomo/ros/quadrotor.hpp"


Quadrotor::Quadrotor(void)
{
    // // wait till connected to FCU
    // this->waitForConnection();
    //
    // // subscribe to topics
    // this->subscribeToPose();
    // this->subscribeToMocap();
    // this->subscribeToIMU();
    //
    // // initialize clients to services
    // this->mode_client = this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
    // this->arming_client = this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);
    //
    // // initialize publishers
    // this->position_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_TOPIC, 50);
    // this->attitude_publisher = this->node.advertise<geometry_msgs::PoseStamped>(ATTITUDE_TOPIC, 50);
    // this->throttle_publisher = this->node.advertise<std_msgs::Float64>(THROTTLE_TOPIC, 50);
    // this->position_controller_x_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_X_CONTROLLER_TOPIC, 50);
    // this->position_controller_y_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_Y_CONTROLLER_TOPIC, 50);
    // this->position_controller_z_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_Z_CONTROLLER_TOPIC, 50);
    //
    // // state
    // this->mission_state = HOVER_MODE;
    //
    // // initialize camera
    // this->tag_timeout = 0;
    // this->cam = new Camera(0, CAMERA_FIREFLY);
    // cam->loadConfig("default", FIREFLY_640);
    // cam->loadConfig("320", FIREFLY_320);
    // cam->loadConfig("160", FIREFLY_160);
    // cam->initCamera("320");
    //
    // // intialize carrot controller
    // double look_ahead_dist = 0.2;
    // double wp_threshold = 0.3;
    // std::deque<Eigen::Vector3d> waypoints;
    // this->carrot_controller = new CarrotController(
    //     waypoints,
    //     look_ahead_dist,
    //     wp_threshold
    // );
    //
    // // initialize position controller
    // this->position_controller = new PositionController(PID_CONFIG);
}

void Quadrotor::poseCallback(const geometry_msgs::PoseStamped &msg)
{
    // position
    this->pose.x = msg.pose.position.x;
    this->pose.y = msg.pose.position.y;
    this->pose.z = msg.pose.position.z;

    // orientation
    quat2euler(
        msg.pose.orientation,
        &this->pose.roll,
        &this->pose.pitch,
        &this->pose.yaw
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
    this->mocap_pose.x = msg.pose.position.x;
    this->mocap_pose.y = msg.pose.position.y;
    this->mocap_pose.z = msg.pose.position.z;

    // mocap orientation
    quat2euler(
        msg.pose.orientation,
        &this->mocap_pose.roll,
        &this->mocap_pose.pitch,
        &this->mocap_pose.yaw
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
    quat2euler(
        msg->orientation,
        &this->pose.roll,
        &this->pose.pitch,
        &this->pose.yaw
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

void Quadrotor::positionControllerCalculate(float x, float y, float z, ros::Time last_request)
{
    float roll;
    float pitch;
    float throttle;
    float roll_adjusted;
    float pitch_adjusted;
    float throttle_adjusted;
    float dt;

    // set x, y, z setpoint and dt
    this->position_controller->x.setpoint = y;
    this->position_controller->y.setpoint = x;
    this->position_controller->T.setpoint = z;
    this->position_controller->dt = (ros::Time::now() - last_request).toSec();

    // calculate new controller inputs
    this->position_controller->calculate(this->pose.x, this->pose.y, this->pose.z, this->pose.yaw);
}

void Quadrotor::printPositionController(void)
{
    ROS_INFO("---");
    ROS_INFO("dt %f", this->position_controller->dt);
    ROS_INFO("quadrotor.pose_x %f", this->pose.x);
    ROS_INFO("quadrotor.pose_y %f", this->pose.y);
    ROS_INFO("quadrotor.pose_z %f", this->pose.z);
    ROS_INFO("quadrotor.pose_yaw %f", rad2deg(this->pose.yaw));
    ROS_INFO("quadrotor.roll %f", rad2deg(this->pose.roll));
    ROS_INFO("quadrotor.pitch %f", rad2deg(this->pose.pitch));
    ROS_INFO("roll.controller %f", rad2deg(this->position_controller->roll));
    ROS_INFO("pitch.controller %f", rad2deg(this->position_controller->pitch));
    ROS_INFO("throttle.controller %f", this->position_controller->throttle);
    ROS_INFO("---");
}

void Quadrotor::buildPositionMessage(
    geometry_msgs::PoseStamped &msg,
    int seq,
    ros::Time time,
    float x,
    float y,
    float z
)
{
    msg.header.seq = seq;
    msg.header.stamp = time;
    msg.header.frame_id = "awesomo_position_cmd";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
}

void Quadrotor::buildAtitudeMessage(
    geometry_msgs::PoseStamped &msg,
    int seq,
    ros::Time time
)
{
    msg.header.seq = seq;
    msg.header.stamp = time;
    msg.header.frame_id = "awesomo_attitude_cmd";
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = this->position_controller->rpy_quat.x();
    msg.pose.orientation.y = this->position_controller->rpy_quat.y();
    msg.pose.orientation.z = this->position_controller->rpy_quat.z();
    msg.pose.orientation.w = this->position_controller->rpy_quat.w();
}

void Quadrotor::buildThrottleMessage(std_msgs::Float64 &msg)
{
    msg.data = this->position_controller->throttle;
}

void Quadrotor::publishPositionControllerStats(int seq, ros::Time time)
{
	geometry_msgs::PoseStamped pid_stats;

    // roll
    pid_stats.header.seq = seq;
    pid_stats.header.stamp = time;
    pid_stats.header.frame_id = "awesomo_position_controller_x";
    pid_stats.pose.position.x = this->position_controller->x.p_error;
    pid_stats.pose.position.y = this->position_controller->x.i_error;
    pid_stats.pose.position.z = this->position_controller->x.d_error;
    this->position_controller_x_publisher.publish(pid_stats);

    // pitch
    pid_stats.header.seq = seq;
    pid_stats.header.stamp = time;
    pid_stats.header.frame_id = "awesomo_position_controller_y";
    pid_stats.pose.position.x = this->position_controller->y.p_error;
    pid_stats.pose.position.y = this->position_controller->y.i_error;
    pid_stats.pose.position.z = this->position_controller->y.d_error;
    this->position_controller_y_publisher.publish(pid_stats);

    // throttle
    pid_stats.header.seq = seq;
    pid_stats.header.stamp = time;
    pid_stats.header.frame_id = "awesomo_position_controller_T";
    pid_stats.pose.position.x = this->position_controller->T.p_error;
    pid_stats.pose.position.y = this->position_controller->T.i_error;
    pid_stats.pose.position.z = this->position_controller->T.d_error;
    this->position_controller_z_publisher.publish(pid_stats);
}

void Quadrotor::buildPositionControllerMessage(
    geometry_msgs::PoseStamped &msg,
    int seq,
    ros::Time time
)
{
	geometry_msgs::PoseStamped attitude;
    std_msgs::Float64 throttle;

    // atitude command
    this->buildAtitudeMessage(attitude, seq, time);
    this->attitude_publisher.publish(attitude);

    // throttle command
    this->buildThrottleMessage(throttle);
    this->throttle_publisher.publish(throttle);
}
