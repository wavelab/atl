#ifndef WAVESIM_ROS_NODES_LZ_NODE_HPP
#define WAVESIM_ROS_NODES_LZ_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include "wave/utils/utils.hpp"
#include "wavesim_ros/utils/node.hpp"
#include "wavesim_gazebo/clients/lz_gclient.hpp"


namespace wavesim {
namespace ros {

using namespace wave;

// NODE SETTINGS
#define NODE_NAME "wavesim_lz"
#define NODE_RATE 100

// PUBLISH TOPICS
#define POSE_RTOPIC "/wavesim/lz/pose"

// SUBSCRIBE TOPICS
#define POSITION_SET_RTOPIC "/wavesim/lz/position/set"
#define VELOCITY_SET_RTOPIC "/wavesim/lz/velocity/set"
#define ANGULAR_VEL_SET_RTOPIC "/wavesim/lz/angular_velocity/set"

class LZNode : public gaz::LZGClient, public ROSNode {
public:
  LZNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  void poseCallback(ConstPosePtr &msg);
  void positionCallback(const geometry_msgs::Vector3 &msg);
  void velocityCallback(const std_msgs::Float64 &msg);
  void angularVelocityCallback(const std_msgs::Float64 &msg);
};

}  // end of ros namespace
}  // end of wavesim namespace
#endif
