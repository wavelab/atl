#ifndef ATL_GAZEBO_BRIDGE_PX4_QUADROTOR_NODE_HPP
#define ATL_GAZEBO_BRIDGE_PX4_QUADROTOR_NODE_HPP

#include <cmath>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "atl/utils/utils.hpp"
#include "atl/ros/utils/node.hpp"
#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/utils.hpp"
#include "atl_gazebo/clients/quadrotor_gclient.hpp"


namespace atl {
namespace ros {

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

class PX4QuadrotorNode : public gaz::QuadrotorGClient, public ROSNode {
public:
  std::string quad_frame;

  PX4QuadrotorNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  void poseCallback(RPYPosePtr &msg);
  void velocityCallback(ConstVector3dPtr &msg);
  void attitudeSetpointCallback(geometry_msgs::PoseStamped msg);
  void throttleSetpointCallback(std_msgs::Float64 msg);
  void positionSetpointCallback(geometry_msgs::PoseStamped msg);
  void velocitySetpointCallback(geometry_msgs::TwistStamped msg);
};

}  // namespace ros
}  // namespace atl
#endif
