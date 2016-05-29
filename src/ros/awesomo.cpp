#include <iostream>
#include <cmath>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>

#include <yaml-cpp/yaml.h>

#include "awesomo/util.hpp"
#include "awesomo/controller.hpp"



// ROS TOPICS
#define IMU_TOPIC "/mavros/imu/data"
#define ARM_TOPIC "/mavros/cmd/arming"
#define MOCAP_TOPIC "/awesomo/mocap/pose"
#define MODE_TOPIC "/mavros/set_mode"
#define POSE_TOPIC "/mavros/local_position/pose"
#define POSITION_TOPIC "/mavros/setpoint_position/local"
#define ATTITUDE_TOPIC "/mavros/setpoint_attitude/attitude"
#define THROTTLE_TOPIC "/mavros/setpoint_attitude/att_throttle"
#define POSITION_X_CONTROLLER_TOPIC "/awesomo/position_controller/x"
#define POSITION_Y_CONTROLLER_TOPIC "/awesomo/position_controller/y"
#define POSITION_Z_CONTROLLER_TOPIC "/awesomo/position_controller/z"
#define RADIO_TOPIC "/mavros/rc/in"
#define LANDING_TOPIC "/awesomo/landing_target/pose"



// STATES
#define IDLE_MODE 0
#define INITIALIZE_MODE 1
#define CARROT_MODE 2
#define TRACKING_MODE 3
#define LAND_MODE 4





class Awesomo
{
private:
    int mission_state;
    mavros_msgs::State state;
    ros::NodeHandle node;

    ros::Subscriber mocap_subscriber;
    ros::Subscriber pose_subscriber;
    ros::Subscriber imu_subscriber;
    ros::Subscriber radio_subscriber;
    ros::Subscriber landing_subscriber;

    ros::ServiceClient mode_client;
    ros::ServiceClient arming_client;

    void poseCallback(const geometry_msgs::PoseStamped &msg);
    void mocapCallback(const geometry_msgs::PoseStamped &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void radioCallback(const mavros_msgs::RCIn &msg);
    void landingCallback(const geometry_msgs::PoseStamped &msg);
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void waitForConnection(void);

public:
    Pose pose;
    Position landing_zone;
    Position landing_zone_prev;
    Position landing_zone_world;
    Pose mocap_pose;
    int rc_in[16];

    CarrotController *carrot_controller;
    PositionController *position_controller;

    ros::Publisher position_publisher;
    ros::Publisher attitude_publisher;
    ros::Publisher throttle_publisher;
    ros::Publisher position_controller_x_publisher;
    ros::Publisher position_controller_y_publisher;
    ros::Publisher position_controller_z_publisher;

    Awesomo(void);
    Awesomo(std::map<std::string, std::string> configs);
    int arm(void);
    int disarm(void);
    int setOffboardModeOn(void);
    void subscribeToPose(void);
    void subscribeToMocap(void);
    void subscribeToIMU(void);
    void subscribeToRadioIn(void);
    void subscribeToLanding(void);
    void positionControllerCalculate(Position p, ros::Time last_request);
    void resetPositionController(void);
    void printPositionController(void);
    void buildPositionMessage(
        geometry_msgs::PoseStamped &msg,
        int seq,
        ros::Time time,
        float x,
        float y,
        float z
    );
    void buildAtitudeMessage(
        geometry_msgs::PoseStamped &msg,
        int seq,
        ros::Time time
    );
    void buildThrottleMessage(std_msgs::Float64 &msg);
    void publishPositionControllerStats(int seq, ros::Time time);
    void publishPositionControllerMessage(
        geometry_msgs::PoseStamped &msg,
        int seq,
        ros::Time time
    );
    void initializeMission(void);
    void runMission(
        geometry_msgs::PoseStamped &msg,
        int seq,
        ros::Time last_request
    );
};

Awesomo::Awesomo(void) {}

Awesomo::Awesomo(std::map<std::string, std::string> configs)
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
    this->landing_zone_prev.x = 0.0;
    this->landing_zone_prev.y = 0.0;
    this->landing_zone_prev.z = 0.0;
    this->landing_zone_world.x = 0.0;
    this->landing_zone_world.y = 0.0;
    this->landing_zone_world.z = 0.0;

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

void Awesomo::poseCallback(const geometry_msgs::PoseStamped &msg)
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

void Awesomo::mocapCallback(const geometry_msgs::PoseStamped &msg)
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

void Awesomo::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    state = *msg;
}

void Awesomo::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    quat2euler(
        msg->orientation,
        &this->pose.roll,
        &this->pose.pitch,
        &this->pose.yaw
    );
}

void Awesomo::radioCallback(const mavros_msgs::RCIn &msg)
{
    for (int i = 0; i < 16; i++){
        this->rc_in[i] = msg.channels[i];
    }
}

void Awesomo::landingCallback(const geometry_msgs::PoseStamped &msg)
{
    this->landing_zone.x = msg.pose.position.x;
    this->landing_zone.y = msg.pose.position.y;
    this->landing_zone.z = msg.pose.position.z;
}

void Awesomo::waitForConnection(void)
{
    ROS_INFO("waiting for FCU ...");
    while (ros::ok() && this->state.connected) {
        ros::spinOnce();
        sleep(1);
    }
    ROS_INFO("connected to FCU!");
}

int Awesomo::arm(void)
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

int Awesomo::disarm(void)
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

int Awesomo::setOffboardModeOn(void)
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

void Awesomo::subscribeToPose(void)
{
    ROS_INFO("subcribing to [POSE]");
    this->pose_subscriber = this->node.subscribe(
        POSE_TOPIC,
        50,
        &Awesomo::poseCallback,
        this
    );
}

void Awesomo::subscribeToMocap(void)
{
    ROS_INFO("subcribing to [MOCAP]");
    this->mocap_subscriber = this->node.subscribe(
        MOCAP_TOPIC,
        50,
        &Awesomo::mocapCallback,
        this
    );
}

void Awesomo::subscribeToIMU(void)
{
    ROS_INFO("subcribing to [IMU_DATA]");
    this->imu_subscriber = this->node.subscribe(
        IMU_TOPIC,
        50,
        &Awesomo::imuCallback,
        this
    );
}

void Awesomo::subscribeToRadioIn(void)
{
    ROS_INFO("subscribing to [RADIO_IN]");
    this->radio_subscriber = this->node.subscribe(
            RADIO_TOPIC,
            50,
            &Awesomo::radioCallback,
            this
     );
}

void Awesomo::subscribeToLanding(void)
{
    ROS_INFO("subcribing to [Landing Zone]");
    this->landing_subscriber = this->node.subscribe(
        LANDING_TOPIC,
        50,
        &Awesomo::landingCallback,
        this
    );
}

void Awesomo::resetPositionController(void)
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

void Awesomo::positionControllerCalculate(Position setpoint, ros::Time last_request)
{
    float dt;

    dt = (ros::Time::now() - last_request).toSec();
    this->position_controller->calculate(setpoint, this->pose, dt);
}

void Awesomo::printPositionController(void)
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

void Awesomo::buildPositionMessage(
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

void Awesomo::buildAtitudeMessage(
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

void Awesomo::buildThrottleMessage(std_msgs::Float64 &msg)
{
    msg.data = this->position_controller->throttle;
}

void Awesomo::publishPositionControllerStats(int seq, ros::Time time)
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

void Awesomo::publishPositionControllerMessage(
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

void Awesomo::initializeMission(void)
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

void Awesomo::runMission(
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
            // this->mission_state = INITIALIZE_MODE;
            this->mission_state = TRACKING_MODE;
            this->landing_zone_prev.x = this->landing_zone.x;
            this->landing_zone_prev.y = this->landing_zone.y;
            this->landing_zone_prev.z = this->landing_zone.z;
            this->landing_zone_world.x = this->pose.x;
            this->landing_zone_world.y = this->pose.y;
            this->landing_zone_world.z = 10;
            // ROS_INFO("TRACKING MODE INITIALIZED");

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
        if (this->landing_zone_prev.x != this->landing_zone.x && this->landing_zone_prev.y != this->landing_zone.y) {
            pos.x = this->pose.x + this->landing_zone.x;
            pos.y = this->pose.y + this->landing_zone.y;
            pos.z = 10;
            this->landing_zone_prev.x = this->landing_zone.x;
            this->landing_zone_prev.y = this->landing_zone.y;
            this->landing_zone_prev.z = this->landing_zone.z;

            this->landing_zone_world.x = pos.x;
            this->landing_zone_world.y = pos.y;
            this->landing_zone_world.z = 10;

        } else {
            pos.x = this->landing_zone_world.x;
            pos.y = this->landing_zone_world.y;
            pos.z = 10;

        }
        ROS_INFO("position: %f\t%f\t%f", pos.x, pos.y, pos.z);
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

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    ros::NodeHandle node_handle;
    ros::Rate rate(50.0);
    ros::Time last_request;
    geometry_msgs::PoseStamped msg;

	int seq = 1;
    Awesomo *awesomo;
    std::string position_controller_config;
    std::string carrot_controller_config;
    std::map<std::string, std::string> configs;

    // get configuration paths
	node_handle.getParam("/position_controller", position_controller_config);
	node_handle.getParam("/carrot_controller", carrot_controller_config);
	configs["position_controller"] = position_controller_config;
	configs["carrot_controller"] = carrot_controller_config;

	// setup awesomo
    ROS_INFO("running ...");
    awesomo = new Awesomo(configs);
    last_request = ros::Time::now();

    while (ros::ok()){
        // awesomo run mission
        awesomo->runMission(msg, seq, last_request);

		// end
		seq++;
        last_request = ros::Time::now();
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
