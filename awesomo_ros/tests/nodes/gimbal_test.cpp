#include <gtest/gtest.h>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/nodes/gimbal_node.hpp"


namespace awesomo {

// NODE SETTNGS
#define TEST_NODE_NAME "gimbal_test_node"

// PUBLISH TOPICS

// SUBSCRIBE TOPICS


class NodeTest : public ::testing::Test {
protected:
  ros::NodeHandle ros_nh;


  NodeTest(void) {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  void poseCallback(const awesomo_msgs::Pose &msg) {
    this->pose_msg = msg;
  }
};

TEST_F(NodeTest, test) {
  ASSERT_TRUE(true);
}

}  // end of awesomo namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, TEST_NODE_NAME);
  return RUN_ALL_TESTS();
}
