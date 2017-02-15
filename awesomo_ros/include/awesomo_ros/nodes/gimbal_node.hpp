#ifndef __AWESOMO_ROS_NODES_GIMBAL_NODE_HPP__
#define __AWESOMO_ROS_NODES_GIMBAL_NODE_HPP__

#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"

namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_gimbal"
#define NODE_RATE 100

// PUBLISH TOPICS
#define CAMERA_IMU_TOPIC "/awesomo/gimbal/joint/imu"
#define FRAME_ORIENTATION_TOPIC "/awesomo/gimbal/frame/orientation/inertial"
#define JOINT_ORIENTATION_TOPIC "/awesomo/gimbal/joint/orientation/inertial"
#define POSITION_TOPIC "/awesomo/gimbal/position/inertial"


// SUBSCRIBE TOPICS
#define SETPOINT_TOPIC "/awesomo/gimbal/setpoint/attitude"
#define TRACK_TOPIC "/awesomo/gimbal/target/track"
#define QUAD_POSE_TOPIC "/mavros/local_position/pose"

#define SHUTDOWN_TOPIC "/awesomo/gimbal/shutdown"


class GimbalNode : public ROSNode {
public:
  std::string quad_frame;
  Gimbal gimbal;
  Vec3 set_points;

  GimbalNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~GimbalNode(void);
  int configure(std::string node_name, int hz);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  void setAttitudeCallback(const geometry_msgs::Vector3 &msg);
  void trackTargetCallback(const geometry_msgs::Vector3 &msg);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
