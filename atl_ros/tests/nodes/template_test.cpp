#include <gtest/gtest.h>

#include "atl/ros/utils/node.hpp"

namespace atl {

// NODE SETTNGS
#define TEST_NODE_NAME "test_node"

// PUBLISH TOPICS

// SUBSCRIBE TOPICS

class NodeTest : public ::testing::Test {
protected:
  ros::NodeHandle ros_nh;

  NodeTest(void) {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
};

TEST_F(NodeTest, test) {
  EXPECT_TRUE(true);
}

}  // namespace atl

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, TEST_NODE_NAME);
  return RUN_ALL_TESTS();
}
