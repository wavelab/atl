#ifndef ATL_GAZEBO_BRIDGE_WORLD_NODE_HPP
#define ATL_GAZEBO_BRIDGE_WORLD_NODE_HPP

#include <stdlib.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <rosgraph_msgs/Clock.h>

#include "atl/utils/utils.hpp"
#include "atl/ros/utils/node.hpp"
#include "atl/ros/utils/msgs.hpp"
#include "atl_gazebo/clients/world_gclient.hpp"

namespace atl {
namespace ros {

// NODE SETTINGS
#define NODE_NAME "atl_world"
#define NODE_RATE 1000

// PUBLISH TOPICS
#define CLOCK_RTOPIC "/clock"

// SUBSCRIBE TOPICS
#define SHUTDOWN_RTOPIC "/atl/world/shutdown"
#define PAUSE_RTOPIC "/atl/world/pause"
#define UNPAUSE_RTOPIC "/atl/world/unpause"
#define RESET_RTOPIC "/atl/world/reset"
#define MODEL_POSE_RTOPIC "/atl/world/model/pose"
#define LOAD_WORLD_RTOPIC "/atl/world/load_world"
#define CLEAR_WORLD_RTOPIC "/atl/world/clear_world"

class WorldNode : public gaz::WorldGClient, public ROSNode {
public:
  WorldNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  void clockCallback(ConstTimePtr &gaz_msg);
  void shutdownCallback(const std_msgs::Bool &msg);
  void pauseCallback(const std_msgs::Bool &msg);
  void unPauseCallback(const std_msgs::Bool &msg);
  void resetCallback(const std_msgs::Bool &msg);
  void modelPoseCallback(const atl_msgs::ModelPose &msg);
  void loadWorldCallback(const std_msgs::String &msg);
  void clearWorldCallback(const std_msgs::Bool &msg);
};

}  // namespace ros
}  // namespace atl
#endif
