#include "awesomo/ros/quadrotor.hpp"

Quadrotor::Quadrotor(void) {}

Quadrotor::Quadrotor(std::map<std::string, std::string> configs)
{
    std::string config_path;

    // state
    this->mission_state = IDLE_MODE;

    // wait till connected to FCU
    this->waitForConnection();

    // initialize clients to services
    this->mode_client = this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
    this->arming_client = this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);

    // initialize publishers
    this->position_publisher = this->node.advertise<geometry_msgs::PoseStamped>(
        POSITION_TOPIC,
        50
    );
    this->attitude_publisher = this->node.advertise<geometry_msgs::PoseStamped>(
        ATTITUDE_TOPIC,
        50
    );
    this->throttle_publisher = this->node.advertise<std_msgs::Float64>(
        THROTTLE_TOPIC,
        50
    );
    this->position_controller_x_publisher = this->node.advertise<geometry_msgs::PoseStamped>(
        POSITION_X_CONTROLLER_TOPIC,
        50
    );
    this->position_controller_y_publisher = this->node.advertise<geometry_msgs::PoseStamped>(
        POSITION_Y_CONTROLLER_TOPIC,
        50
    );
    this->position_controller_z_publisher = this->node.advertise<geometry_msgs::PoseStamped>(
        POSITION_Z_CONTROLLER_TOPIC,
        50
    );

    // initialize subscribers
	this->subscribeToPose();
	this->subscribeToRadioIn();
	this->subscribeToLanding();

    // initialize rc_in[16] array
    for (int i = 0; i < 16; i++) {
        this->rc_in[i] = 0.0f;
    }

    // initialize landing zone
    this->landing_zone.x = 0.0;
    this->landing_zone.y = 0.0;
    this->landing_zone.z = 0.0;

    // initialize controllers
    if (configs.count("position_controller")) {
        config_path = configs["position_controller"];
        this->position_controller = new PositionController(config_path);
        ROS_INFO("position controller initialized!");
    } else {
        this->position_controller = NULL;
    }

    if (configs.count("carrot_controller")) {
        config_path = configs["carrot_controller"];
        this->carrot_controller = new CarrotController(config_path);
        ROS_INFO("carrot controller initialized!");
    } else {
        this->carrot_controller = NULL;
    }
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

void Quadrotor::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    state = *msg;
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

void Quadrotor::radioCallback(const mavros_msgs::RCIn &msg)
{
    for (int i = 0; i < 16; i++){
        this->rc_in[i] = msg.channels[i];
    }
}

void Quadrotor::landingCallback(const geometry_msgs::PoseStamped &msg)
{
    this->landing_zone.x = msg.pose.position.x;
    this->landing_zone.y = msg.pose.position.y;
    this->landing_zone.z = msg.pose.position.z;
}

void Quadrotor::waitForConnection(void)
{
    ROS_INFO("waiting for FCU ...");
    while (ros::ok() && this->state.connected) {
        ros::spinOnce();
        sleep(1);
    }
    ROS_INFO("connected to FCU!");
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

void Quadrotor::subscribeToRadioIn(void)
{
    ROS_INFO("subscribing to [RADIO_IN]");
    this->radio_subscriber = this->node.subscribe(
            RADIO_TOPIC,
            50,
            &Quadrotor::radioCallback,
            this
     );
}

void Quadrotor::subscribeToLanding(void)
{
    ROS_INFO("subcribing to [Landing Zone]");
    this->landing_subscriber = this->node.subscribe(
        LANDING_TOPIC,
        50,
        &Quadrotor::landingCallback,
        this
    );
}

void Quadrotor::resetPositionController(void)
{
    this->position_controller->x.sum_error = 0.0;
    this->position_controller->x.prev_error = 0.0;
    this->position_controller->x.output = 0.0;

    this->position_controller->y.sum_error = 0.0;
    this->position_controller->y.prev_error = 0.0;
    this->position_controller->y.output = 0.0;

    this->position_controller->T.sum_error = 0.0;
    this->position_controller->T.prev_error = 0.0;
    this->position_controller->T.output = 0.0;
}

void Quadrotor::positionControllerCalculate(Position setpoint, ros::Time last_request)
{
    float dt;

    dt = (ros::Time::now() - last_request).toSec();
    this->position_controller->calculate(setpoint, this->pose, dt);
}

void Quadrotor::printPositionController(void)
{
    ROS_INFO("---");
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
    pid_stats.pose.orientation.x = this->position_controller->x.output * M_PI / 180;
    pid_stats.pose.orientation.y = this->position_controller->x.setpoint;
    this->position_controller_x_publisher.publish(pid_stats);

    // pitch
    pid_stats.header.seq = seq;
    pid_stats.header.stamp = time;
    pid_stats.header.frame_id = "awesomo_position_controller_y";
    pid_stats.pose.position.x = this->position_controller->y.p_error;
    pid_stats.pose.position.y = this->position_controller->y.i_error;
    pid_stats.pose.position.z = this->position_controller->y.d_error;
    pid_stats.pose.orientation.x = this->position_controller->y.output * M_PI / 180;
    pid_stats.pose.orientation.y = this->position_controller->y.setpoint;
    this->position_controller_y_publisher.publish(pid_stats);

    // throttle
    pid_stats.header.seq = seq;
    pid_stats.header.stamp = time;
    pid_stats.header.frame_id = "awesomo_position_controller_T";
    pid_stats.pose.position.x = this->position_controller->T.p_error;
    pid_stats.pose.position.y = this->position_controller->T.i_error;
    pid_stats.pose.position.z = this->position_controller->T.d_error;
    pid_stats.pose.orientation.x = this->position_controller->T.output * M_PI / 180;
    pid_stats.pose.orientation.y = this->position_controller->T.setpoint;
    this->position_controller_z_publisher.publish(pid_stats);
}

void Quadrotor::publishPositionControllerMessage(
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

void Quadrotor::initializeMission(void)
{
    Position pos;
    Eigen::Vector3d wp;
    Eigen::Vector3d carrot;
    Eigen::Vector3d position;

    // current position + some altitude
    pos.x = this->pose.x;
    pos.y = this->pose.y;
    pos.z = this->pose.z + 3;

    // waypoint 1
    wp << pos.x, pos.y, pos.z;
    this->carrot_controller->wp_start = wp;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 2
    wp << pos.x + 5, pos.y, pos.z;
    this->carrot_controller->wp_end = wp;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 3
    wp << pos.x + 5, pos.y + 5, pos.z;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 4
    wp << pos.x, pos.y + 5, pos.z;
    this->carrot_controller->waypoints.push_back(wp);

    // back to waypoint 1
    wp << pos.x, pos.y, pos.z;
    this->carrot_controller->waypoints.push_back(wp);

    // initialize carrot controller
    this->carrot_controller->initialized = 1;
}

void Quadrotor::runMission(
    geometry_msgs::PoseStamped &msg,
    int seq,
    ros::Time last_request
)
{
    Position pos;
    Eigen::Vector3d position;
    Eigen::Vector3d carrot;

    switch (this->mission_state) {
    case IDLE_MODE:
        // check if offboard switch has been turned on
        if (this->rc_in[6] < 1500) {
            this->resetPositionController();

            // configure setpoint to be where the quad currently is
            pos.x = this->pose.x;
            pos.y = this->pose.y;
            pos.z = this->pose.z + 3;

        } else {
            // transition to offboard mode
            this->mission_state = INITIALIZE_MODE;
            // this->mission_state = TRACKING_MODE;

        }
        break;

    case INITIALIZE_MODE:
        this->initializeMission();
        std::cout << "Carrot controller initialized!" << std::endl;
        this->mission_state = CARROT_MODE;
        break;

    case CARROT_MODE:
        position << this->pose.x, this->pose.y, this->pose.z;
        if (this->carrot_controller->update(position, carrot) == 0) {
            ROS_INFO("Landing!");
            pos.x = this->pose.x;
            pos.y = this->pose.y;
            pos.z = this->pose.z - 2;
            this->mission_state = LAND_MODE;

        } else {
            pos.x = carrot(0);
            pos.y = carrot(1);
            pos.z = carrot(2);

        }

        break;

    case TRACKING_MODE:
        pos.x = this->landing_zone.x;
        pos.y = this->landing_zone.y;
        pos.z = 5;
        break;

    case LAND_MODE:
        // do nothing
        break;
    }

    // publish quadrotor position controller
    this->positionControllerCalculate(pos, last_request);
    this->publishPositionControllerMessage(msg, seq, ros::Time::now());
    this->publishPositionControllerStats(seq, ros::Time::now());
}
