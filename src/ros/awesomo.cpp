#include <iostream>
#include <cmath>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>

#include <atim/AtimPoseStamped.h>

#include "awesomo/util.hpp"
#include "awesomo/controller.hpp"
#include "awesomo/quadrotor.hpp"
#include "awesomo/PositionControllerStats.h"



// ROS TOPICS
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



class Awesomo
{
public:
    ros::NodeHandle node;
    mavros_msgs::State state;

    Pose pose;
    Pose mocap_pose;
    LandingTargetPosition landing_zone;
    bool landing_zone_detected;
    int rc_in[16];
    Quadrotor *quad;

    ros::ServiceClient mode_client;
    ros::ServiceClient arming_client;

    ros::Subscriber mocap_subscriber;
    ros::Subscriber pose_subscriber;
    ros::Subscriber radio_subscriber;
    ros::Subscriber landing_subscriber;

    ros::Publisher position_publisher;
    ros::Publisher attitude_publisher;
    ros::Publisher throttle_publisher;
    ros::Publisher position_controller_x_publisher;
    ros::Publisher position_controller_y_publisher;
    ros::Publisher position_controller_z_publisher;

    void poseCallback(const geometry_msgs::PoseStamped &msg);
    void mocapCallback(const geometry_msgs::PoseStamped &msg);
    void radioCallback(const mavros_msgs::RCIn &msg);
    void landingCallback(const atim::AtimPoseStamped &msg);
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void waitForConnection(void);

    Awesomo(std::map<std::string, std::string> configs);
    int arm(void);
    int disarm(void);
    int setOffboardModeOn(void);
    void subscribeToPose(void);
    void subscribeToMocap(void);
    void subscribeToRadioIn(void);
    void subscribeToLanding(void);
    void publishPositionControllerStats(int seq, ros::Time time);
    void publishPositionControllerMessage(
        geometry_msgs::PoseStamped &msg,
        int seq,
        ros::Time time
    );
    void run(
        geometry_msgs::PoseStamped &msg,
        int seq,
        ros::Time last_request
    );
};

Awesomo::Awesomo(std::map<std::string, std::string> configs)
{
    std::string config_path;

    // state
    for (int i = 0; i < 16; i++) {
        this->rc_in[i] = 0.0f;
    }

    quad = new Quadrotor(configs);

    // wait till connected to FCU
    this->waitForConnection();

    // initialize clients to services
    this->mode_client = this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
    this->arming_client = this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);

    // initialize subscribers
	this->subscribeToPose();
	this->subscribeToRadioIn();
	this->subscribeToLanding();

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

void Awesomo::radioCallback(const mavros_msgs::RCIn &msg)
{
    for (int i = 0; i < 16; i++){
        this->rc_in[i] = msg.channels[i];
    }
}

void Awesomo::landingCallback(const atim::AtimPoseStamped &msg)
{
    this->landing_zone.detected = msg.tag_detected;
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

void Awesomo::publishPositionControllerStats(int seq, ros::Time time)
{
	geometry_msgs::PoseStamped msg;

    // roll
    msg.header.seq = seq;
    msg.header.stamp = time;
    msg.header.frame_id = "awesomo_position_controller_x";
    msg.pose.position.x = this->quad->position_controller->x.p_error;
    msg.pose.position.y = this->quad->position_controller->x.i_error;
    msg.pose.position.z = this->quad->position_controller->x.d_error;
    msg.pose.orientation.x = this->quad->position_controller->x.output * M_PI / 180;
    msg.pose.orientation.y = this->quad->position_controller->x.setpoint;
    this->position_controller_x_publisher.publish(msg);

    // pitch
    msg.header.seq = seq;
    msg.header.stamp = time;
    msg.header.frame_id = "awesomo_position_controller_y";
    msg.pose.position.x = this->quad->position_controller->y.p_error;
    msg.pose.position.y = this->quad->position_controller->y.i_error;
    msg.pose.position.z = this->quad->position_controller->y.d_error;
    msg.pose.orientation.x = this->quad->position_controller->y.output * M_PI / 180;
    msg.pose.orientation.y = this->quad->position_controller->y.setpoint;
    this->position_controller_y_publisher.publish(msg);

    // throttle
    msg.header.seq = seq;
    msg.header.stamp = time;
    msg.header.frame_id = "awesomo_position_controller_T";
    msg.pose.position.x = this->quad->position_controller->T.p_error;
    msg.pose.position.y = this->quad->position_controller->T.i_error;
    msg.pose.position.z = this->quad->position_controller->T.d_error;
    msg.pose.orientation.x = this->quad->position_controller->T.output * M_PI / 180;
    msg.pose.orientation.y = this->quad->position_controller->T.setpoint;
    this->position_controller_z_publisher.publish(msg);
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
    attitude.header.seq = seq;
    attitude.header.stamp = time;
    attitude.header.frame_id = "awesomo_attitude_cmd";
    attitude.pose.position.x = 0;
    attitude.pose.position.y = 0;
    attitude.pose.position.z = 0;
    attitude.pose.orientation.x = this->quad->position_controller->rpy_quat.x();
    attitude.pose.orientation.y = this->quad->position_controller->rpy_quat.y();
    attitude.pose.orientation.z = this->quad->position_controller->rpy_quat.z();
    attitude.pose.orientation.w = this->quad->position_controller->rpy_quat.w();
    this->attitude_publisher.publish(attitude);

    // throttle command
    throttle.data = this->quad->position_controller->throttle;
    this->throttle_publisher.publish(throttle);
}

void Awesomo::run(
    geometry_msgs::PoseStamped &msg,
    int seq,
    ros::Time last_request
)
{
    float dt;

    // calculate attitude from position controller
    dt = (ros::Time::now() - last_request).toSec();
    this->quad->runMission(this->pose, this->landing_zone, dt);

    // publish quadrotor position controller
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

	float dt;
	int seq;
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
    seq = 1;
    ROS_INFO("running ...");
    awesomo = new Awesomo(configs);
    last_request = ros::Time::now();

    while (ros::ok()){
        // check if offboard switch has been turned on
        if (awesomo->rc_in[6] < 1500) {
            awesomo->quad->resetPositionController();
        } else {
            awesomo->run(msg, seq, last_request);
        }

		// end
		seq++;
        last_request = ros::Time::now();
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
