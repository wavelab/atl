#include "awesomo_ros/gimbal_node.hpp"

namespace awesomo {

int GimbalNode::configure(std::string node_name, int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // gimbal
  this->ros_nh->getParam("/gimbal_config", config_file);
  if (this->gimbal.configure(config_file) != 0) {
    ROS_ERROR("Failed to configure Gimal!");
    return -2;
  };

  // loop callback
  this->registerLoopCallback(std::bind(&GimbalNode::loopCallback, this));

  this->configured = true;
  return 0;
}

int GimbalNode::loopCallback(void) {
  return 0;
}

}  // end of awesomo namespace


int main(int argc, char **argv) {
  awesomo::GimbalNode node;

  if (node.configure(GIMBAL_NODE_NAME, GIMBAL_NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure GimbalNode!");
    return -1;
  }
  node.loop();

  return 0;
}
