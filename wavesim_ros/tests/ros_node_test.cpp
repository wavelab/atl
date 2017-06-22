#include <std_msgs/String.h>

#include "wavesim_ros/utils/node.hpp"


namespace wavesim {
namespace ros {

class ROSNodeTest : public ROSNode {
public:
  ROSNodeTest(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  int loopCallback(void);
  void sayCallback(const std_msgs::String &msg);
};

int ROSNodeTest::configure(const std::string &node_name, int hz) {
  ROSNode::configure(node_name, hz);

  // clang-format off
  ROSNode::registerLoopCallback(std::bind(&ROSNodeTest::loopCallback, this));
  ROSNode::registerPublisher<std_msgs::String>("ros_node_test/say");
  ROSNode::registerSubscriber("ros_node_test/say", &ROSNodeTest::sayCallback, this);
  // clang-format on

  return 0;
}

int ROSNodeTest::loopCallback(void) {
  std_msgs::String msg;
  msg.data = "Hello";
  this->ros_pubs["ros_node_test/say"].publish(msg);
  return 0;
}

void ROSNodeTest::sayCallback(const std_msgs::String &msg) {
  std::cout << msg.data << std::endl;
}

}  // end of ros namespace
}  // end of wavesim namespace

int main(int argc, char **argv) {
  wavesim::ros::ROSNodeTest node(argc, argv);
  node.configure("ros_node_test", 1);
  node.loop();
  return 0;
}
