#include "awesomo_ros/apriltag_node.hpp"

namespace awesomo {

int AprilTagNode::configure(const std::string &node_name, int hz) {
  ROSNode::configure(node_name, hz);

  // clang-format off
  ROSNode::registerPublisher<std_msgs::String>(SAY_TOPIC);
  ROSNode::registerSubscriber(SAY_TOPIC, &AprilTagNode::sayCallback, this);
  ROSNode::registerLoopCallback(std::bind(&AprilTagNode::loopCallback, this));
  // clang-format on

  return 0;
}

int AprilTagNode::loopCallback(void) {
  std_msgs::String msg;
  msg.data = "Hello";
  this->ros_pubs[SAY_TOPIC].publish(msg);
  return 0;
}

void AprilTagNode::sayCallback(const std_msgs::String &msg) {
  std::cout << msg.data << std::endl;
}

}  // end of awesomo namespace

int main(void) {
  awesomo::AprilTagNode node;
  node.configure(NODE_NAME, NODE_RATE);
  node.loop();
  return 0;
}
