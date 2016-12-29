#include "awesomo_ros/gimbal_node.hpp"

namespace awesomo {

GimbalNode::GimbalNode(void) {
  this->configured = false;
}

int GimbalNode::configure(std::string node_name, int hz) {
  std::string config_file;

  // ros node
  ROSNode::configure(node_name, hz);

  // gimbal
  this->ros_nh->getParam("/config_file", config_file);
  // if (this->gimbal.configure(config_file) != 0) {
  //   return -1;
  // };

  // register loop callback
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

  // configure and loop
  if (node.configure(NODE_NAME, NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure GimbalNode!");
    return -1;
  }
  node.loop();

  return 0;
}
