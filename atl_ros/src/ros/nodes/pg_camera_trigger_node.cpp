#include "atl/ros/nodes/pg_camera_trigger_node.hpp"

namespace atl {

int PGCameraTriggerNode::configure(const int hz) {
  std::string config_path;
  std::string guid_str;

  // ros node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  // get params
  ROS_GET_PARAM(this->node_name + "/guid", guid_str);
  this->guid = std::stoull(guid_str); // ros param cannot parse a uint64_t
  ROS_GET_PARAM(this->node_name + "/config_dir", config_path);

  // configure camera
  if (this->camera.configure(config_path) != 0) {
    ROS_ERROR("Failed to configure camera!");
    return -2;
  }

  // connect camerea
  if (this->camera.connect(this->guid) != 0) {
    ROS_ERROR("Failed to connect to camera!");
    return -3;
  }

  // register loop callback
  this->addLoopCallback(std::bind(&PGCameraTriggerNode::loopCallback, this));

  this->configured = true;
  return 0;
}

int PGCameraTriggerNode::loopCallback() {
  if (this->camera.trigger() != 0) {
    ROS_ERROR("Failed to trigger camera!");
    return -1;
  }
  LOG_INFO("Trigger camera!");

  return 0;
}

} // namespace atl

RUN_ROS_NODE(atl::PGCameraTriggerNode, NODE_RATE);
