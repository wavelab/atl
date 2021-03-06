#include <gtest/gtest.h>

#include <sensor_msgs/Imu.h>

#include "atl/ros/utils/node.hpp"

namespace atl {

// NODE SETTNGS
#define TEST_NODE_NAME "gimbal_test_node"

// SUBSCRIBE TOPICS
#define FRAME_JOINT_TOPIC "/atl/gimbal/joint/imu"
#define FRAME_ORIENTATION_TOPIC "/atl/gimbal/frame/orientiation/inertial"

// PUBLISH TOPICS
#define SET_ATTITUDE_TOPIC "/atl/gimbal/setpoint/attitude"

class NodeTest : public ::testing::Test {
protected:
  ros::NodeHandle ros_nh;

  ros::Subscriber joint_W_imu_sub;
  ros::Subscriber frame_W_orientation_sub;

  ros::Publisher setpoint_pub;

  sensor_msgs::Imu joint_W_imu_msg;
  geometry_msgs::Quaternion frame_W_orien_msg;

  NodeTest() {
    // Subscribers
    // clang-format off
    this->joint_W_imu_sub = this->ros_nh.subscribe(FRAME_JOINT_TOPIC, 1, &NodeTest::jointImuCallback, this);
    this->frame_W_orientation_sub = this->ros_nh.subscribe(FRAME_ORIENTATION_TOPIC, 1, &NodeTest::frameOrientationCallback, this);
    // clang-format on

    // Publishers
    this->setpoint_pub =
        this->ros_nh.advertise<geometry_msgs::Vector3>(SET_ATTITUDE_TOPIC, 1);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  virtual void SetUp() {
    geometry_msgs::Vector3 setpoint_msg;
    setpoint_msg.x = 10;
    setpoint_msg.y = 10;
    setpoint_msg.z = 0; // Yaw is not supported in this gimbal

    this->setpoint_pub.publish(setpoint_msg);

    // spin and sleep
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  void jointImuCallback(const sensor_msgs::Imu &msg) {
    this->joint_W_imu_msg = msg;
  }

  void frameOrientationCallback(const geometry_msgs::Quaternion &msg) {
    this->frame_W_orien_msg = msg;
  }
};

TEST_F(NodeTest, jointImuMessage) {
  EXPECT_EQ(1, this->joint_W_imu_sub.getNumPublishers());
  // ASSERT_NEAR(0.0, this->pos_W_msg.x, 0.1);
  // ASSERT_NEAR(0.0, this->pos_W_msg.y, 0.1);
  // ASSERT_NEAR(0.0, this->pos_W_msg.z, 0.1);
}

TEST_F(NodeTest, frameOrientationMsg) {
  EXPECT_EQ(1, this->frame_W_orientation_sub.getNumPublishers());
  // ASSERT_NEAR(0.0, this->pos_W_msg.x, 0.1);
  // ASSERT_NEAR(0.0, this->pos_W_msg.y, 0.1);
  // ASSERT_NEAR(0.0, this->pos_W_msg.z, 0.1);
}

} // namespace atl

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, TEST_NODE_NAME);
  return RUN_ALL_TESTS();
}
