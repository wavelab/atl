#ifndef atl_ROS_NODES_GIMBAL_NODE_HPP
#define atl_ROS_NODES_GIMBAL_NODE_HPP

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "atl_ros/utils/node.hpp"
#include "atl_gazebo/clients/gimbal_gclient.hpp"


namespace atl {
namespace ros {

using namespace wave;

#define ROLL_JOINT "quad_with_gimbal_cam::gimbal_camera::roll_motor_joint"
#define PITCH_JOINT "quad_with_gimbal_cam::gimbal_camera::pitch_motor_joint"

// NODE SETTINGS
#define NODE_NAME "atl_gimbal"
#define NODE_RATE 100

// PUBLISH TOPICS
#define FRAME_ORIENTATION_RTOPIC "/atl/gimbal/frame/orientation/inertial"
#define JOINT_ORIENTATION_RTOPIC "/atl/gimbal/joint/orientation/inertial"
#define POSITION_RTOPIC "/atl/gimbal/position/inertial"

// SUBSCRIBE TOPICS
#define POSE_RTOPIC "/atl/quadrotor/pose/local"
#define SETPOINT_RTOPIC "/atl/gimbal/setpoint/attitude"
#define TRACK_RTOPIC "/atl/gimbal/target/track"

// COORDINATE SYSTEM
class GimbalNode : public gaz::GimbalGClient, public ROSNode {
public:
  std::string quad_frame;
  bool enable_tracking;

  GimbalNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  void frameOrientationCallback(ConstQuaternionPtr &msg);
  void jointOrientationCallback(ConstQuaternionPtr &msg);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  void setAttitudeCallback(const geometry_msgs::Vector3 msg);
  void trackTargetCallback(const geometry_msgs::Vector3 msg);
};

}  // end of ros namespace
}  // end of atl namespace
#endif
