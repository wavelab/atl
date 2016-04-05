#ifndef __QUADROTOR_NODE_HPP__
#define __QUADROTOR_NODE_HPP__

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

#include "awesomo/util.hpp"
#include "awesomo/camera.hpp"


// CONSTANTS
#define IMU_TOPIC "/mavros/imu/data"
#define ARM_TOPIC "/mavros/cmd/arming"
#define MOCAP_TOPIC "/awesomo/mocap/pose"
#define MODE_TOPIC "/mavros/set_mode"
#define POSE_TOPIC "/mavros/local_position/pose"
#define POSITION_TOPIC "/mavros/setpoint_position/local"
#define ATTITUDE_TOPIC "/mavros/setpoint_attitude/attitude"
#define THROTTLE_TOPIC "/mavros/setpoint_attitude/att_throttle"

#define HOVER_MODE 0
#define DISCOVER_MODE 1
#define PLANNING_MODE 2
#define CARROT_MODE 3


struct pose
{
    double x;
    double y;
    double z;

    double roll;
    double pitch;
    double yaw;
};

class Quadrotor
{
    private:
        int mission_state;
        mavros_msgs::State state;
        ros::NodeHandle node;

        ros::Subscriber mocap_subscriber;
        ros::Subscriber pose_subscriber;
        ros::Subscriber imu_subscriber;

        ros::ServiceClient mode_client;
        ros::ServiceClient arming_client;

        Camera *cam;
        int tag_timeout;
        std::vector<TagPose> tag_poses;
        Eigen::Vector3d tag_position;

        void poseCallback(const geometry_msgs::PoseStamped &msg);
        void subscribeToPose(void);
        void mocapCallback(const geometry_msgs::PoseStamped &msg);
        void subscribeToMocap(void);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void subscribeToIMU(void);
        void stateCallback(const mavros_msgs::State::ConstPtr &msg);
        void waitForConnection(void);

    public:
        double roll;
        double pitch;
        double yaw;

        double pose_x;
        double pose_y;
        double pose_z;

        double pose_roll;
        double pose_pitch;
        double pose_yaw;

        double mocap_x;
        double mocap_y;
        double mocap_z;

        double mocap_roll;
        double mocap_pitch;
        double mocap_yaw;

        ros::Publisher position_publisher;
        ros::Publisher attitude_publisher;
        ros::Publisher throttle_publisher;

        Quadrotor(void);
        int arm(void);
        int disarm(void);
        int setOffboardModeOn(void);
        void runMission(geometry_msgs::PoseStamped &pose);
};

#endif
