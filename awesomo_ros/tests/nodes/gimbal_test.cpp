#include <gtest/gtest.h>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/nodes/gimbal_node.hpp"


namespace awesomo {

// NODE SETTNGS
#define TEST_NODE_NAME "gimbal_test_node"

// SUBSCRIBE TOPICS
#define CAMERA_ORIENTATION_TOPIC "/awesomo/gimbal/joint/orientiation/inertial"
#define FRAME_ORIENTATION_TOPIC "/awesomo/gimbal/frame/orientiation/inertial"
#define ACC_TOPIC "/awesomo/gimbal/joint/accel"
#define GYRO_TOPIC "/awesomo/gimbal/joint/gyro"

// PUBLISH TOPICS
#define SET_ATTITUDE_TOPIC "/awesomo/gimbal/setpoint/attitude"


class NodeTest : public ::testing::Test {
protected:
  ros::NodeHandle ros_nh;

  ros::Subscriber joint_if_orientation_sub;
  ros::Subscriber joint_if_acc_sub;
  ros::Subscriber joint_if_gyro_sub;
  ros::Subscriber frame_if_orientation_sub;

  ros::Publisher setpoint_pub;

  geometry_msgs::Quaternion joint_if_orien_msg;
  geometry_msgs::Quaternion frame_if_orien_msg;
  geometry_msgs::Vector3 joint_accel_msg;
  geometry_msgs::Vector3 joint_gyro_msg;

  NodeTest(void) {
    // clang-format off

    // Subscribers
    this->joint_if_orientation_sub = this->ros_nh.subscribe(CAMERA_ORIENTATION_TOPIC, 1, &NodeTest::CameraOrientationCallback, this);
    this->join_if_gyro_sub = this->ros_nh.subscribe(GYRO_TOPIC, 1, &NodeTest::gyroCallback, this);
    this->join_if_acc_sub = this->ros_nh.subscribe(ACC_TOPIC, 1, &NodeTest::AccCallback, this);
    this->frame_if_orientation_sub = this->ros_nh.subscribe(FRAME_ORIENTATION_TOPIC, 1, &NodeTestCameraOrientationCallback, this);

    // Publishers
    this->setpoint_pub = this->ros_nh.advertise<geometry_msgs::Vector3>(SET_ATTITIUDE_TOPIC, 1);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  virtual void SetUp(void) {

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

  void cameraOrientationCallback(const geometry_msgs::Quaterion msg) {
    this->joint_if_orien_msg = msg;
  }

  void frameOrientationCallback(const geometry_msgs::Quaterion msg) {
    this->frame_if_orien_msg = msg;
  }

  void accCallback(const geometry_msgs::Quaterion msg) {
    this->joint_accel_msg = msg;
  }
  void gryoCallback(const geometry_msgs::Quaterion msg) {
    this->joint_gyro_msg = msg;
  }

};

TEST_F(NodeTest, jointOrientationMsg) {
  ASSERT_EQ(1, this->joint_if_orientation_sub.getNumPublishers());
  // ASSERT_NEAR(0.0, this->pos_if_msg.x, 0.1);
  // ASSERT_NEAR(0.0, this->pos_if_msg.y, 0.1);
  // ASSERT_NEAR(0.0, this->pos_if_msg.z, 0.1);
}

TEST_F(NodeTest, frameOrientationMsg) {
  ASSERT_EQ(1, this->frame_if_orientation_sub.getNumPublishers());
  // ASSERT_NEAR(0.0, this->pos_if_msg.x, 0.1);
  // ASSERT_NEAR(0.0, this->pos_if_msg.y, 0.1);
  // ASSERT_NEAR(0.0, this->pos_if_msg.z, 0.1);
}

TEST_F(NodeTest, gyroMsg) {
  ASSERT_EQ(1, this->joint_if_gyro_sub.getNumPublishers());
  // ASSERT_NEAR(0.0, this->pos_if_msg.x, 0.1);
  // ASSERT_NEAR(0.0, this->pos_if_msg.y, 0.1);
  // ASSERT_NEAR(0.0, this->pos_if_msg.z, 0.1);
}

TEST_F(NodeTest, accMsg) {
  ASSERT_EQ(1, this->joint_if_acc_sub.getNumPublishers());
  // ASSERT_NEAR(0.0, this->pos_if_msg.x, 0.1);
  // ASSERT_NEAR(0.0, this->pos_if_msg.y, 0.1);
  // ASSERT_NEAR(0.0, this->pos_if_msg.z, 0.1);
}


}  // end of awesomo namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, TEST_NODE_NAME);
  return RUN_ALL_TESTS();
}
