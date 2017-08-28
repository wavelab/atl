#ifndef ATL_ROS_NODES_PG_CAMERA_TRIGGER_NODE_HPP
#define ATL_ROS_NODES_PG_CAMERA_TRIGGER_NODE_HPP

#include <ros/ros.h>

#include "atl/atl_core.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 1;

namespace atl {

class PGCameraTriggerNode : public ROSNode {
public:
  DC1394Camera camera;
  uint64_t guid = 0;

  PGCameraTriggerNode(int argc, char **argv) : ROSNode(argc, argv) {}

  int configure(const int hz);
  int loopCallback();
};

} // namespace atl
#endif
