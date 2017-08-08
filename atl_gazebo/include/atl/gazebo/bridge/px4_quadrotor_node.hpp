#ifndef ATL_GAZEBO_BRIDGE_PX4_QUADROTOR_NODE_HPP
#define ATL_GAZEBO_BRIDGE_PX4_QUADROTOR_NODE_HPP

#include <cmath>

#include <Eigen/Geometry>

#include <ros/ros.h>

#include "atl/gazebo/clients/quadrotor_gclient.hpp"
#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"
#include "atl/ros/utils/utils.hpp"
#include "atl/utils/utils.hpp"

namespace atl {
namespace gazebo_bridge {

// NODE SETTINGS
#define NODE_NAME "atl_px4_quadrotor"
#define NODE_RATE 200

// PUBLISH TOPICS
#define PX4_POSE_RTOPIC "/mavros/local_position/pose"
#define PX4_VELOCITY_RTOPIC "/mavros/local_position/velocity"

// SUBSCRIBE TOPICS
#define PX4_RADIO_RTOPIC "/mavros/rc/in"
#define PX4_MODE_RTOPIC "/mavros/set_mode"
#define PX4_ARM_RTOPIC "/mavros/cmd/arming"
#define PX4_ATTITUDE_SETPOINT_RTOPIC "/mavros/setpoint_attitude/attitude"
#define PX4_THROTTLE_SETPOINT_RTOPIC "/mavros/setpoint_attitude/att_throttle"
#define PX4_POSITION_SETPOINT_RTOPIC "/mavros/setpoint_position/local"
#define PX4_VELOCITY_SETPOINT_RTOPIC "/mavros/setpoint_velocity/cmd_vel"

/** PX4 Quadrotor ROS Node */
class PX4QuadrotorNode : public gaz::QuadrotorGClient, public ROSNode {
public:
  std::string quad_frame;

  PX4QuadrotorNode(int argc, char **argv) : ROSNode(argc, argv) {}

  /**
   * Configure
   * @param node_name Name of ROS node
   * @param hz ROS node rate in hertz
   */
  int configure(const std::string &node_name, int hz);

  /**
   * Quadrotor pose Gazebo callback
   * @param msg Roll, pitch and yaw message
   */
  void poseCallback(const RPYPosePtr &msg);

  /**
   * Quadrotor velocity Gazebo callback
   * @param msg Velocity message in x, y, z
   */
  void velocityCallback(const ConstVector3dPtr &msg);

  /**
   * Quadrotor attitude setpoint ROS callback
   * @param msg Pose message
   */
  void attitudeSetpointCallback(const geometry_msgs::PoseStamped &msg);

  /**
   * Quadrotor throttle setpoint ROS callback
   * @param msg Throttle message
   */
  void throttleSetpointCallback(const std_msgs::Float64 &msg);

  /**
   * Quadrotor position setpoint ROS callback
   * @param msg Pose message
   */
  void positionSetpointCallback(const geometry_msgs::PoseStamped &msg);

  /**
   * Quadrotor velocity setpoint ROS callback
   * @param msg Twist message
   */
  void velocitySetpointCallback(const geometry_msgs::TwistStamped &msg);
};

}  // namespace gazebo_bridge
}  // namespace atl
#endif
