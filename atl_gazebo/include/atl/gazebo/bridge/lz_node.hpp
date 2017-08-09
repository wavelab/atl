#ifndef ATL_GAZEBO_BRIDGE_LZ_NODE_HPP
#define ATL_GAZEBO_BRIDGE_LZ_NODE_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "atl/gazebo/clients/lz_gclient.hpp"
#include "atl/ros/utils/node.hpp"
#include "atl/utils/utils.hpp"

namespace atl {
namespace gazebo_bridge {

// NODE SETTINGS
#define NODE_NAME "atl_lz"
#define NODE_RATE 100

// PUBLISH TOPICS
#define POSE_RTOPIC "/atl/lz/pose"

// SUBSCRIBE TOPICS
#define POSITION_SET_RTOPIC "/atl/lz/position/set"
#define VELOCITY_SET_RTOPIC "/atl/lz/velocity/set"
#define ANGULAR_VEL_SET_RTOPIC "/atl/lz/angular_velocity/set"

class LZNode : public gaz::LZGClient, public ROSNode {
public:
  LZNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(int hz);
  void poseCallback(ConstPosePtr &msg);
  void positionCallback(const geometry_msgs::Vector3 &msg);
  void velocityCallback(const std_msgs::Float64 &msg);
  void angularVelocityCallback(const std_msgs::Float64 &msg);
};

}  // namespace gazebo_bridge
}  // namespace atl
#endif
