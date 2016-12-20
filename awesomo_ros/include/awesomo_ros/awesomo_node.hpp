#ifndef __AWESOMO_ROS_AWESOMO_NODE_HPP__
#define __AWESOMO_ROS_AWESOMO_NODE_HPP__

#include <iostream>
#include <cmath>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>

#include <atim/AtimPoseStamped.h>

#include <awesomo_msgs/PositionControllerStats.h>
#include <awesomo_msgs/KFStats.h>
#include <awesomo_msgs/KFPlotting.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/ros_node.hpp"


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
#define ATTITUDE_RPY_TOPIC "/awesomo/setpoint_attitude/attitude_rpy"


namespace awesomo {

class AwesomoNode : public ROSNode {
public:
  // ros::NodeHandle node;
  // mavros_msgs::State state;

  Pose world_pose;
  // Pose hover_point;
  // Velocity velocity;
  // Pose mocap_pose;
  // Pose gps_pose;
  // LandingTargetPosition landing_zone;
  // bool landing_zone_detected;
  int rc_in[16];

  Quadrotor quad;

  // ros::ServiceClient mode_client;
  // ros::ServiceClient arming_client;
  //
  // ros::Subscriber mocap_subscriber;
  // ros::Subscriber world_pose_subscriber;
  // ros::Subscriber velocity_subscriber;
  // ros::Subscriber radio_subscriber;
  // ros::Subscriber landing_subscriber;
  // ros::Subscriber gps_subscriber;
  //
  // ros::Publisher position_publisher;
  // ros::Publisher attitude_publisher;
  // ros::Publisher attitude_rpy_publisher;
  // ros::Publisher throttle_publisher;
  // ros::Publisher position_controller_stats_publisher;
  // ros::Publisher kf_estimator_stats_publisher;
  // ros::Publisher kf_estimator_stats_plotting_publisher;
  //
  // tf::TransformBroadcaster world_imu_tf_broadcaster;
  // tf::Transform world_imu_tf;

  AwesomoNode(void);
  int configure(std::map<std::string, std::string> configs);

  void poseCallback(const geometry_msgs::PoseStamped &msg);
  // void velocityCallback(const geometry_msgs::TwistStamped &msg);
  // void mocapCallback(const geometry_msgs::PoseStamped &msg);
  // void radioCallback(const mavros_msgs::RCIn &msg);
  // void atimCallback(const atim::AtimPoseStamped &msg);
  // void gpsCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
  // void stateCallback(const mavros_msgs::State::ConstPtr &msg);
  // void waitForConnection(void);

  // int arm(void);
  // int disarm(void);
  // int setOffboardModeOn(void);
  // void subscribeToPose(void);
  // void subscribeToVelocity(void);
  // void subscribeToMocap(void);
  // void subscribeToRadioIn(void);
  // void subscribeToAtim(void);
  // void subscribeToGPS(void);
  // void publishHoverCommand(int seq, ros::Time time);
  // void publishPositionControllerStats(int seq, ros::Time time);
  // void publishPositionControllerMessage(geometry_msgs::PoseStamped &msg,
  //                                       int seq,
  //                                       ros::Time time);
  // void publishKFStats(int seq, ros::Time time);
  // void publishKFStatsForPlotting(int seq, ros::Time time);
  // int run(geometry_msgs::PoseStamped &msg, int seq, ros::Time
  // last_request);
};

}  // end of awesomo namespace
#endif
