#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>

#include "atl/ros/utils/node.hpp"

namespace atl {

// NODE SETTNGS
#define TEST_NODE_NAME "estimator_test_node"

// PUBLISH TOPICS
#define QUAD_POSE_TOPIC "/mavros/local_position/pose"
#define QUAD_VELOCITY_TOPIC "/mavros/local_position/velocity"
#define TARGET_TOPIC "/atl/apriltag/target/inertial"

// SUBSCRIBE TOPICS
#define POS_W_TOPIC "/atl/estimate/landing_target/position/inertial"
#define VEL_W_TOPIC "/atl/estimate/landing_target/velocity/inertial"
#define POS_B_TOPIC "/atl/estimate/landing_target/position/body"
#define VEL_B_TOPIC "/atl/estimate/landing_target/velocity/body"
#define DETECTED_TOPIC "/atl/estimate/landing_target/detected"
#define GIMBAL_TOPIC "/atl/gimbal/setpoint/attitude"

class NodeTest : public ::testing::Test {
protected:
  ros::NodeHandle ros_nh;

  ros::Subscriber pos_W_sub;
  ros::Subscriber vel_W_sub;
  ros::Subscriber pos_B_sub;
  ros::Subscriber vel_B_sub;
  ros::Subscriber detected_sub;
  ros::Subscriber gimbal_sub;

  ros::Publisher quad_pose_pub;
  ros::Publisher quad_vel_pub;
  ros::Publisher target_pub;

  geometry_msgs::Vector3 pos_W_msg;
  geometry_msgs::Vector3 vel_W_msg;
  geometry_msgs::Vector3 pos_B_msg;
  geometry_msgs::Vector3 vel_B_msg;
  bool tag_detected;
  geometry_msgs::Vector3 gimbal_msg;

  NodeTest() {
    // clang-format off
    this->pos_W_sub = this->ros_nh.subscribe(POS_W_TOPIC, 200, &NodeTest::inertialPositionCallback, this);
    this->vel_W_sub = this->ros_nh.subscribe(VEL_W_TOPIC, 200, &NodeTest::inertialVelocityCallback, this);
    this->pos_B_sub = this->ros_nh.subscribe(POS_B_TOPIC, 200, &NodeTest::bodyPositionCallback, this);
    this->vel_B_sub = this->ros_nh.subscribe(VEL_B_TOPIC, 200, &NodeTest::bodyVelocityCallback, this);
    this->detected_sub = this->ros_nh.subscribe(DETECTED_TOPIC, 200, &NodeTest::detectedCallback, this);
    this->gimbal_sub = this->ros_nh.subscribe(GIMBAL_TOPIC, 200, &NodeTest::gimbalCallback, this);

    this->quad_pose_pub = this->ros_nh.advertise<geometry_msgs::PoseStamped>(QUAD_POSE_TOPIC, 1);
    this->quad_vel_pub = this->ros_nh.advertise<geometry_msgs::TwistStamped>(QUAD_VELOCITY_TOPIC, 1);
    this->target_pub = this->ros_nh.advertise<geometry_msgs::Vector3>(TARGET_TOPIC, 1);
    // clang-format on

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  virtual void SetUp() {
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped vel_msg;
    geometry_msgs::Vector3 target_msg;

    // intialize nb_detected
    this->tag_detected = false;

    // quadrotor pose
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 3.0;

    pose_msg.pose.orientation.w = 1.0;
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;

    // quadrotor velocity
    vel_msg.twist.linear.x = 0.0;
    vel_msg.twist.linear.y = 0.0;
    vel_msg.twist.linear.z = 0.0;

    // target
    target_msg.x = 0.0;
    target_msg.y = 0.0;
    target_msg.z = 0.0;

    this->target_pub.publish(target_msg);
    this->quad_pose_pub.publish(pose_msg);
    this->quad_vel_pub.publish(vel_msg);

    // spin and sleep
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  void inertialPositionCallback(const geometry_msgs::Vector3 &msg) {
    this->pos_W_msg = msg;
  }

  void inertialVelocityCallback(const geometry_msgs::Vector3 &msg) {
    this->vel_W_msg = msg;
  }

  void bodyPositionCallback(const geometry_msgs::Vector3 &msg) {
    this->pos_B_msg = msg;
  }

  void bodyVelocityCallback(const geometry_msgs::Vector3 &msg) {
    this->vel_B_msg = msg;
  }

  void detectedCallback(const std_msgs::Bool &msg) {
    if (msg.data == true) {
      std::cout << msg.data << std::endl;
      this->tag_detected = true;
    }
  }

  void gimbalCallback(const geometry_msgs::Vector3 &msg) {
    this->gimbal_msg = msg;
  }
};

TEST_F(NodeTest, inertialPosition) {
  EXPECT_EQ(1, this->pos_W_sub.getNumPublishers());
  ASSERT_NEAR(0.0, this->pos_W_msg.x, 0.1);
  ASSERT_NEAR(0.0, this->pos_W_msg.y, 0.1);
  ASSERT_NEAR(0.0, this->pos_W_msg.z, 0.1);
}

TEST_F(NodeTest, inertialVelocity) {
  EXPECT_EQ(1, this->vel_W_sub.getNumPublishers());
  ASSERT_NEAR(0.0, this->vel_W_msg.x, 0.1);
  ASSERT_NEAR(0.0, this->vel_W_msg.y, 0.1);
  ASSERT_NEAR(0.0, this->vel_W_msg.z, 0.1);
}

TEST_F(NodeTest, bodyPosition) {
  EXPECT_EQ(1, this->pos_B_sub.getNumPublishers());
  ASSERT_NEAR(0.0, this->pos_B_msg.x, 0.1);
  ASSERT_NEAR(0.0, this->pos_B_msg.y, 0.1);
  ASSERT_NEAR(-3.0, this->pos_B_msg.z, 0.1);
}

TEST_F(NodeTest, bodyVelocity) {
  EXPECT_EQ(1, this->vel_B_sub.getNumPublishers());
  ASSERT_NEAR(0.0, this->vel_B_msg.x, 0.1);
  ASSERT_NEAR(0.0, this->vel_B_msg.y, 0.1);
  ASSERT_NEAR(0.0, this->vel_B_msg.z, 0.1);
}

TEST_F(NodeTest, detected) {
  EXPECT_EQ(1, this->detected_sub.getNumPublishers());
  EXPECT_TRUE(this->tag_detected);
}

TEST_F(NodeTest, gimbal) {
  EXPECT_EQ(1, this->gimbal_sub.getNumPublishers());
  ASSERT_NEAR(0.0, this->gimbal_msg.x, 0.1);
  ASSERT_NEAR(0.0, this->gimbal_msg.y, 0.1);
  ASSERT_NEAR(0.0, this->gimbal_msg.z, 0.1);
}

} // namespace atl

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, TEST_NODE_NAME);
  return RUN_ALL_TESTS();
}
