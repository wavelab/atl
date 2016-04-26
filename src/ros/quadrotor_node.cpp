#include "awesomo/ros/quadrotor_node.hpp"


Quadrotor::Quadrotor(void)
{
    // wait till connected to FCU
    this->waitForConnection();

    // subscribe to topics
    this->subscribeToPose();
    this->subscribeToMocap();
    this->subscribeToIMU();

    // initialize clients to services
    this->mode_client = this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
    this->arming_client = this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);

    // initialize publishers
    this->position_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_TOPIC, 50);
    this->attitude_publisher = this->node.advertise<geometry_msgs::PoseStamped>(ATTITUDE_TOPIC, 50);
    this->throttle_publisher = this->node.advertise<std_msgs::Float64>(THROTTLE_TOPIC, 50);
    this->position_controller_x_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_X_CONTROLLER_TOPIC, 50);
    this->position_controller_y_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_Y_CONTROLLER_TOPIC, 50);
    this->position_controller_z_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_Z_CONTROLLER_TOPIC, 50);

    // state
    this->mission_state = HOVER_MODE;

    // initialize camera
    this->tag_timeout = 0;
    this->cam = new Camera(0, CAMERA_FIREFLY);
    cam->loadConfig("default", FIREFLY_640);
    cam->loadConfig("320", FIREFLY_320);
    cam->loadConfig("160", FIREFLY_160);
    cam->initCamera("320");

    // intialize carrot controller
    double look_ahead_dist = 0.2;
    double wp_threshold = 0.3;
    std::deque<Eigen::Vector3d> waypoints;
    this->carrot_controller = new CarrotController(
        waypoints,
        look_ahead_dist,
        wp_threshold
    );

    // initialize position controller
    this->position_controller = new PositionController(PID_CONFIG);
}

void Quadrotor::poseCallback(const geometry_msgs::PoseStamped &msg)
{
    //  mocap position
    this->pose_x = msg.pose.position.x;
    this->pose_y = msg.pose.position.y;
    this->pose_z = msg.pose.position.z;

    //  mocap orientation
    quat2euler(msg.pose.orientation, &this->pose_roll, &this->pose_pitch, &this->pose_yaw);

    // print
    // ROS_INFO(
    //     "GOT POSE: [roll: %f, pitch: %f, yaw: %f, x: %f, y: %f, z: %f]",
    //     rad2deg(this->pose_roll),
    //     rad2deg(this->pose_pitch),
    //     rad2deg(this->pose_yaw),
    //     pose_x,
    //     pose_y,
    //     pose_z
    // );
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
    // ROS_INFO(
    //     "GOT MOCAP: [%f, %f, %f]",
    //     rad2deg(this->mocap_roll),
    //     rad2deg(this->mocap_pitch),
    //     rad2deg(this->mocap_yaw)
    // );
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
    // ROS_INFO(
    //     "GOT IMU: [%f, %f, %f]",
    //     rad2deg(this->roll),
    //     rad2deg(this->pitch),
    //     rad2deg(this->yaw)
    // );
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
    ros::Duration dt;

    // configure x, y, z setpoint
    this->position_controller->x.setpoint = y;
    this->position_controller->y.setpoint = x;
    this->position_controller->T.setpoint = z;

    // position controller - calculate
    this->position_controller->dt = ros::Time::now() - last_request;
    dt = this->position_controller->dt;
    pid_calculate(&this->position_controller->x, this->pose_y, dt);
    pid_calculate(&this->position_controller->y, this->pose_x, dt);
    pid_calculate(&this->position_controller->T, this->pose_z, dt);
    roll = -this->position_controller->x.output;
    pitch = this->position_controller->y.output;
    throttle = this->position_controller->T.output;

    // adjust roll and pitch according to yaw
    if (this->pose_yaw < 0) {
        this->pose_yaw += 2 * M_PI;
    }
    roll_adjusted = cos(this->pose_yaw) * roll - sin(this->pose_yaw) * pitch;
    pitch_adjusted = sin(this->pose_yaw) * roll + cos(this->pose_yaw) * pitch;

    // throttle
    throttle_adjusted = this->position_controller->hover_throttle + throttle;
    throttle_adjusted = throttle_adjusted / (cos(roll_adjusted) * cos(pitch_adjusted));

    // update position controller
    this->position_controller->roll = roll_adjusted;
    this->position_controller->pitch = pitch_adjusted;
    this->position_controller->rpy_quat = euler2quat(roll_adjusted, pitch_adjusted, 0);
    this->position_controller->throttle = throttle_adjusted;
}

void Quadrotor::printPositionController(void)
{
    ROS_INFO("---");
    ROS_INFO("dt %f", this->position_controller->dt.toSec());
    ROS_INFO("quadrotor.pose_x %f", this->pose_x);
    ROS_INFO("quadrotor.pose_y %f", this->pose_y);
    ROS_INFO("quadrotor.pose_z %f", this->pose_z);
    ROS_INFO("quadrotor.pose_yaw %f", rad2deg(this->pose_yaw));
    ROS_INFO("quadrotor.roll %f", rad2deg(this->roll));
    ROS_INFO("quadrotor.pitch %f", rad2deg(this->pitch));
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

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    ros::Time now;
    ros::Time last_request;
    ros::Rate rate(50.0);  // publishing rate MUST be faster than 2Hz
    Quadrotor quad;

    ROS_INFO("running ...");
	int seq = 1;
	int index = 0;

    while (ros::ok()){
        // quad.runMission(position);
        last_request = ros::Time::now();
        now = last_request;

		// publish


		// end
		seq++;
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
