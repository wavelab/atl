#include <iostream>
#include <cmath>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>

#include <atim/AtimPoseStamped.h>

#include "awesomo/util.hpp"
#include "awesomo/camera_mount.hpp"
#include "awesomo/controller.hpp"
#include "awesomo/quadrotor.hpp"
#include "awesomo/PositionControllerStats.h"
#include "awesomo/KFStats.h"
#include "awesomo/KFPlotting.h"



// ROS TOPICS
#define ARM_TOPIC "/mavros/cmd/arming"
#define MOCAP_TOPIC "/awesomo/mocap/pose"
#define MODE_TOPIC "/mavros/set_mode"
#define POSE_TOPIC "/mavros/local_position/pose"
#define VELOCITY_TOPIC "/mavros/local_position/velocity"
#define POSITION_TOPIC "/mavros/setpoint_position/local"
#define ATTITUDE_TOPIC "/mavros/setpoint_attitude/attitude"
#define THROTTLE_TOPIC "/mavros/setpoint_attitude/att_throttle"
#define POSITION_CONTROLLER_TOPIC "/awesomo/position_controller/stats"
#define KF_ESTIMATION_TOPIC "/awesomo/kf_estimation/stats"
#define KF_ESTIMATION_PLOTTING_TOPIC "/awesomo/kf_estimation/states"
#define RADIO_TOPIC "/mavros/rc/in"
#define GPS_TOPIC "/mavros/global_position/local"
#define ATIM_POSE_TOPIC "/atim/pose"


class Awesomo
{
public:
    ros::NodeHandle node;
    mavros_msgs::State state;

    Pose world_pose;
    Pose hover_point;
    Velocity velocity;
    Pose mocap_pose;
    Pose gps_pose;
    LandingTargetPosition landing_zone;
    bool landing_zone_detected;
    int rc_in[16];

    Quadrotor *quad;
    CameraMount *camera_mount;

    ros::ServiceClient mode_client;
    ros::ServiceClient arming_client;

    ros::Subscriber mocap_subscriber;
    ros::Subscriber world_pose_subscriber;
    ros::Subscriber velocity_subscriber;
    ros::Subscriber radio_subscriber;
    ros::Subscriber landing_subscriber;
    ros::Subscriber gps_subscriber;

    ros::Publisher position_publisher;
    ros::Publisher attitude_publisher;
    ros::Publisher throttle_publisher;
    ros::Publisher position_controller_stats_publisher;
    ros::Publisher kf_estimator_stats_publisher;
    ros::Publisher kf_estimator_stats_plotting_publisher;

    tf::TransformBroadcaster world_imu_tf_broadcaster;
    tf::Transform world_imu_tf;

    void poseCallback(const geometry_msgs::PoseStamped &msg);
    void velocityCallback(const geometry_msgs::TwistStamped &msg);
    void mocapCallback(const geometry_msgs::PoseStamped &msg);
    void radioCallback(const mavros_msgs::RCIn &msg);
    void atimCallback(const atim::AtimPoseStamped &msg);
    void gpsCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void waitForConnection(void);
    void imuCallback(const geometry_msgs::PoseStamped &msg);

    Awesomo(std::map<std::string, std::string> configs);
    int arm(void);
    int disarm(void);
    int setOffboardModeOn(void);
    void subscribeToPose(void);
    void subscribeToVelocity(void);
    void subscribeToMocap(void);
    void subscribeToRadioIn(void);
    void subscribeToAtim(void);
    void subscribeToGPS(void);
    void publishHoverCommand(int seq, ros::Time time);
    void publishPositionControllerStats(int seq, ros::Time time);
    void publishPositionControllerMessage(
        geometry_msgs::PoseStamped &msg,
        int seq,
        ros::Time time
    );
    void publishKFStats(int seq, ros::Time time);
    void publishKFStatsForPlotting(int seq, ros::Time time);
    int run(
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

    this->quad = new Quadrotor(configs);
    this->camera_mount = new CameraMount(0.0, deg2rad(-90), 0.0, 0.0, 0.0, 0.0);

    // wait till connected to FCU
    this->waitForConnection();

    // initialize clients to services
    this->mode_client = this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
    this->arming_client = this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);

    // initialize subscribers
	this->subscribeToPose();
	this->subscribeToVelocity();
	this->subscribeToRadioIn();
	this->subscribeToAtim();

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
    this->position_controller_stats_publisher = this->node.advertise<awesomo::PositionControllerStats>(
        POSITION_CONTROLLER_TOPIC,
        50
    );
    this->kf_estimator_stats_publisher = this->node.advertise<awesomo::KFStats>(
        KF_ESTIMATION_TOPIC,
        50
    );
    this->kf_estimator_stats_plotting_publisher = this->node.advertise<awesomo::KFPlotting>(
        KF_ESTIMATION_PLOTTING_TOPIC,
        50
    );
}

void Awesomo::poseCallback(const geometry_msgs::PoseStamped &msg)
{
    Eigen::Quaterniond quat;
    Eigen::Vector3d position;

    quat = Eigen::Quaterniond(
        msg.pose.orientation.w,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z
    );

    position << msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z + this->quad->height_offset;

    // set height offset
    // NOTE: this assumes the quadrotor starts off from the ground
    if (this->quad->height_offset_initialized == false) {
        this->quad->height_offset = 0.0 - msg.pose.position.z;
        this->quad->height_offset_initialized = true;
        printf("ASSUMING QUADROTOR IS ON THE GROUND!!!\n");
        printf("height_offset: %f\n", this->quad->height_offset);
    }

    // ENU coordinates
    this->world_pose = Pose(quat, position);

    this->world_imu_tf.setOrigin(
        tf::Vector3(
            this->world_pose.position(0),
            this->world_pose.position(1),
            this->world_pose.position(2)
        )
    );

    // for display in rviz, serves no other function
    this->world_imu_tf.setRotation(
        tf::Quaternion(
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        )
    );

    this->world_imu_tf_broadcaster.sendTransform(
            tf::StampedTransform(world_imu_tf, ros::Time::now(), "world", "pixhawk"));
}

void Awesomo::velocityCallback(const geometry_msgs::TwistStamped &msg)
{
    this->velocity.linear_x = msg.twist.linear.x;
    this->velocity.linear_y = msg.twist.linear.y;
    this->velocity.linear_z = msg.twist.linear.z;

    this->velocity.angular_x = msg.twist.angular.x;
    this->velocity.angular_y = msg.twist.angular.y;
    this->velocity.angular_z = msg.twist.angular.z;
}

void Awesomo::mocapCallback(const geometry_msgs::PoseStamped &msg)
{
    // mocap position
    Eigen::Quaterniond quat;
    Eigen::Vector3d position;

    quat = Eigen::Quaterniond(
        msg.pose.orientation.w,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z
    );

    position << msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z;

    this->mocap_pose = Pose(quat, position);
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

void Awesomo::atimCallback(const atim::AtimPoseStamped &msg)
{
    Eigen::Vector3d tag;
    Eigen::Vector3d tag_BPF;
    Eigen::Quaterniond imu;
    geometry_msgs::Quaternion q;

    tag << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    q = msg.pose.orientation;
    imu = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    tag_BPF = this->camera_mount->getTargetPositionBPFrame(tag, imu);

    this->landing_zone.detected = msg.tag_detected;
    this->landing_zone.position << tag_BPF(0), tag_BPF(1), tag_BPF(2);
}

void Awesomo::gpsCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    Eigen::Quaterniond quat;
    Eigen::Vector3d position;

    quat = Eigen::Quaterniond(
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z
    );

    position << msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z;

    this->gps_pose = Pose(quat, position);
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
    this->world_pose_subscriber = this->node.subscribe(
        POSE_TOPIC,
        50,
        &Awesomo::poseCallback,
        this
    );
}

void Awesomo::subscribeToVelocity(void)
{
    ROS_INFO("subcribing to [VELOCITY]");
    this->velocity_subscriber = this->node.subscribe(
        VELOCITY_TOPIC,
        50,
        &Awesomo::velocityCallback,
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

void Awesomo::subscribeToAtim(void)
{
    ROS_INFO("subcribing to [ATIM]");
    this->landing_subscriber = this->node.subscribe(
        ATIM_POSE_TOPIC,
        50,
        &Awesomo::atimCallback,
        this
    );
}

void Awesomo::subscribeToGPS(void)
{
    ROS_INFO("subcribing to [GPS UTM]");
    this->gps_subscriber = this->node.subscribe(
        GPS_TOPIC,
        50,
        &Awesomo::gpsCallback,
        this
    );
}

void Awesomo::publishHoverCommand(int seq, ros::Time time)
{
	geometry_msgs::PoseStamped hover_cmd;
	float adjusted_height;

	// setup
	adjusted_height = this->quad->hover_height + this->quad->height_offset;

    // msg header
    hover_cmd.header.seq = seq;
    hover_cmd.header.stamp = time;
    hover_cmd.header.frame_id = "awesomo_hover_position";

    // set pose
    hover_cmd.pose.position.x = this->hover_point.position(0);
    hover_cmd.pose.position.y = this->hover_point.position(1);
    hover_cmd.pose.position.z = adjusted_height;
    hover_cmd.pose.orientation.w = this->hover_point.q.w();
    hover_cmd.pose.orientation.x = this->hover_point.q.x();
    hover_cmd.pose.orientation.y = this->hover_point.q.y();
    hover_cmd.pose.orientation.z = this->hover_point.q.z();

    this->position_publisher.publish(hover_cmd);
}

void Awesomo::publishPositionControllerStats(int seq, ros::Time time)
{
	awesomo::PositionControllerStats msg;

    // msg header
    msg.header.seq = seq;
    msg.header.stamp = time;
    msg.header.frame_id = "awesomo_position_controller";

    // roll
    msg.roll_p_error = this->quad->position_controller->x.p_error;
    msg.roll_i_error = this->quad->position_controller->x.i_error;
    msg.roll_d_error = this->quad->position_controller->x.d_error;
    msg.roll_output = this->quad->position_controller->roll * 180 / M_PI;
    msg.roll_setpoint  = this->quad->position_controller->x.setpoint;

    // pitch
    msg.pitch_p_error = this->quad->position_controller->y.p_error;
    msg.pitch_i_error = this->quad->position_controller->y.i_error;
    msg.pitch_d_error = this->quad->position_controller->y.d_error;
    msg.pitch_output = this->quad->position_controller->pitch * 180 / M_PI;
    msg.pitch_setpoint  = this->quad->position_controller->pitch;

    // thrust
    msg.throttle_p_error = this->quad->position_controller->T.p_error;
    msg.throttle_i_error = this->quad->position_controller->T.i_error;
    msg.throttle_d_error = this->quad->position_controller->T.d_error;
    msg.throttle_output = this->quad->position_controller->T.output;
    msg.throttle_setpoint = this->quad->position_controller->T.setpoint;

    this->position_controller_stats_publisher.publish(msg);
}

void Awesomo::publishPositionControllerMessage(
    geometry_msgs::PoseStamped &msg,
    int seq,
    ros::Time time
)
{
	geometry_msgs::PoseStamped attitude;
    std_msgs::Float64 throttle;
    PositionController *position_controller;

    // setup
    position_controller = this->quad->position_controller;

    // atitude command
    attitude.header.seq = seq;
    attitude.header.stamp = time;
    attitude.header.frame_id = "awesomo_attitude_cmd";
    attitude.pose.position.x = 0;
    attitude.pose.position.y = 0;
    attitude.pose.position.z = 0;
    attitude.pose.orientation.x = position_controller->command_quat.x();
    attitude.pose.orientation.y = position_controller->command_quat.y();
    attitude.pose.orientation.z = position_controller->command_quat.z();
    attitude.pose.orientation.w = position_controller->command_quat.w();
    this->attitude_publisher.publish(attitude);

    // throttle command
    throttle.data = this->quad->position_controller->throttle;
    this->throttle_publisher.publish(throttle);
}

void Awesomo::publishKFStats(int seq, ros::Time time)
{
	awesomo::KFStats msg;
	struct kf *estimator;

	// setup
	estimator = &this->quad->tag_estimator;

    // message header
    msg.header.seq = seq;
    msg.header.stamp = time;
    msg.header.frame_id = "awesomo_kf_estimation";

    // A matrix
    msg.A_rows = estimator->A.rows();
    msg.A_cols = estimator->A.cols();
    for (int i = 0; i < estimator->A.rows(); i++) {
        for (int j = 0; j < estimator->A.cols(); j++) {
            msg.A_data[i] = estimator->A(i, j);
        }
    }

    // B matrix
    msg.B_rows = estimator->B.rows();
    msg.B_cols = estimator->B.cols();
    for (int i = 0; i < estimator->B.rows(); i++) {
        for (int j = 0; j < estimator->B.cols(); j++) {
            msg.A_data[i] = estimator->B(i, j);
        }
    }

    // R matrix
    msg.R_rows = estimator->R.rows();
    msg.R_cols = estimator->R.cols();
    for (int i = 0; i < estimator->R.rows(); i++) {
        for (int j = 0; j < estimator->R.cols(); j++) {
            msg.A_data[i] = estimator->R(i, j);
        }
    }

    // C matrix
    msg.C_rows = estimator->C.rows();
    msg.C_cols = estimator->C.cols();
    for (int i = 0; i < estimator->C.rows(); i++) {
        for (int j = 0; j < estimator->C.cols(); j++) {
            msg.A_data[i] = estimator->C(i, j);
        }
    }

    // Q matrix
    msg.Q_rows = estimator->Q.rows();
    msg.Q_cols = estimator->Q.cols();
    for (int i = 0; i < estimator->Q.rows(); i++) {
        for (int j = 0; j < estimator->Q.cols(); j++) {
            msg.A_data[i] = estimator->Q(i, j);
        }
    }

    // S matrix
    msg.S_rows = estimator->S.rows();
    msg.S_cols = estimator->S.cols();
    for (int i = 0; i < estimator->S.rows(); i++) {
        for (int j = 0; j < estimator->S.cols(); j++) {
            msg.A_data[i] = estimator->S(i, j);
        }
    }

    // K matrix
    msg.K_rows = estimator->K.rows();
    msg.K_cols = estimator->K.cols();
    for (int i = 0; i < estimator->K.rows(); i++) {
        for (int j = 0; j < estimator->K.cols(); j++) {
            msg.A_data[i] = estimator->K(i, j);
        }
    }

    // mu vector
    msg.mu_size = estimator->mu.size();
    for (int i = 0; i < estimator->mu.size(); i++) {
        msg.mu_data[i] = estimator->mu(i);
    }

    // mu vector
    msg.mu_p_size = estimator->mu_p.size();
    for (int i = 0; i < estimator->mu_p.size(); i++) {
        msg.mu_p_data[i] = estimator->mu_p(i);
    }

    // S_p matrix
    msg.S_p_rows = estimator->S_p.rows();
    msg.S_p_cols = estimator->S_p.cols();
    for (int i = 0; i < estimator->S_p.rows(); i++) {
        for (int j = 0; j < estimator->S_p.cols(); j++) {
            msg.A_data[i] = estimator->S_p(i, j);
        }
    }

    this->kf_estimator_stats_publisher.publish(msg);
}

void Awesomo::publishKFStatsForPlotting(int seq, ros::Time time)
{
    awesomo::KFPlotting msg;
    struct kf *estimator;

    // setup
    if (this->quad->estimator_initialized) {
        estimator = &this->quad->tag_estimator;

        // message header
        msg.header.seq = seq;
        msg.header.stamp = time;
        msg.header.frame_id = "awesomo_kf_estimation";

        // mu vector
        msg.x = estimator->mu(0);
        msg.y = estimator->mu(1);
        msg.z = estimator->mu(2);

        msg.vel_x = estimator->mu(3);
        msg.vel_y = estimator->mu(4);
        msg.vel_z = estimator->mu(5);

        msg.acc_x = estimator->mu(6);
        msg.acc_y = estimator->mu(7);
        msg.acc_z = estimator->mu(8);

        // publish
        this->kf_estimator_stats_plotting_publisher.publish(msg);
    }
}

int Awesomo::run(
    geometry_msgs::PoseStamped &msg,
    int seq,
    ros::Time last_request
)
{
    float dt;
    int retval;

    // calculate attitude from position controller
    dt = (ros::Time::now() - last_request).toSec();

    // run mission
    retval = this->quad->runMission(this->world_pose, this->landing_zone, dt);
    if (retval == MISSION_ACCOMPLISHED) {
        this->disarm();
        return 0;

    } else if (retval == DISCOVER_MODE) {
        this->quad->resetPositionController();
        this->publishHoverCommand(seq, ros::Time::now());
        return 1;

    } else {
        this->hover_point = this->world_pose;  // keep track of hover point
        this->publishPositionControllerMessage(msg, seq, ros::Time::now());
        this->publishPositionControllerStats(seq, ros::Time::now());
        this->publishKFStatsForPlotting(seq, ros::Time::now());
        return 1;
    }

}

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    ros::NodeHandle node_handle;
    ros::Rate rate(100.0);
    ros::Time last_request;
    geometry_msgs::PoseStamped msg;

	float dt;
	int seq;
    Awesomo *awesomo;
    std::string quadrotor_config;
    std::string position_controller_config;
    std::string carrot_controller_config;
    std::map<std::string, std::string> configs;
    std_msgs::Float64 throttle;

    // get configuration paths
	node_handle.getParam("/quadrotor", quadrotor_config);
	node_handle.getParam("/position_controller", position_controller_config);
	node_handle.getParam("/carrot_controller", carrot_controller_config);
	configs["quadrotor"] = quadrotor_config;
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
            throttle.data = 0.0;
            awesomo->hover_point = awesomo->world_pose;
            awesomo->quad->mission_state = DISCOVER_MODE;
            awesomo->quad->resetPositionController();
            awesomo->throttle_publisher.publish(throttle);

        } else {
            if (awesomo->run(msg, seq, last_request) == 0) {
                return 0;
            }

        }

		// end
		seq++;
        last_request = ros::Time::now();
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
