#ifndef ATL_GAZEBO_BRIDGE_WORLD_NODE_HPP
#define ATL_GAZEBO_BRIDGE_WORLD_NODE_HPP

#include <stdlib.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>

#include "atl/gazebo/clients/world_gclient.hpp"
#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"
#include "atl/utils/utils.hpp"

namespace atl {
namespace gazebo_bridge {

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

/** Gazebo ROS Node */
class WorldNode : public gaz::WorldGClient, public ROSNode {
public:
  WorldNode(int argc, char **argv) : ROSNode(argc, argv) {}

  /**
   * Configure
   * @param node_name Name of ROS Node
   * @param hz ROS node rate in hertz
   */
  int configure(const std::string &node_name, int hz);

  /**
   * Clock Gazebo callback
   * @param msg Time message
   */
  void clockCallback(ConstTimePtr &gaz_msg);

  /**
   * Shutdown ROS callback
   * @param msg Bool message
   */
  void shutdownCallback(const std_msgs::Bool &msg);

  /**
   * Pause ROS callback
   * @param msg Bool message
   */
  void pauseCallback(const std_msgs::Bool &msg);

  /**
   * UnPause ROS callback
   * @param msg Bool message
   */
  void unPauseCallback(const std_msgs::Bool &msg);

  /**
   * Reset ROS callback
   * @param msg Bool message
   */
  void resetCallback(const std_msgs::Bool &msg);

  /**
   * Model pose ROS callback
   * @param msg Model pose message
   */
  void modelPoseCallback(const atl_msgs::ModelPose &msg);

  /**
   * Load world ROS callback
   * @param msg String message
   */
  void loadWorldCallback(const std_msgs::String &msg);

  /**
   * Clear world ROS callback
   * @param msg Bool message
   */
  void clearWorldCallback(const std_msgs::Bool &msg);
};

}  // namespace gazebo_bridge
}  // namespace atl
#endif
