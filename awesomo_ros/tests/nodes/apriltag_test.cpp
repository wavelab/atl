#include <gtest/gtest.h>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/nodes/apriltag_node.hpp"

namespace awesomo {

// NODE SETTNGS
#define TEST_NODE_NAME "apriltag_test_node"

// PUBLISH TOPICS
#define SHUTDOWN_TOPIC "/awesomo/apriltag/shutdown"
#define IMAGE_TOPIC "/awesomo/camera/image_pose_stamped"

// SUBSCRIBE TOPICS
#define TARGET_POSE_TOPIC "/awesomo/apriltag/target"
#define TARGET_IF_TOPIC "/awesomo/apriltag/target/inertial"
#define TARGET_BPF_TOPIC "/awesomo/apriltag/target/body"

// TEST DATA
#define TEST_IMAGE "test_data/image.jpg"

class NodeTest : public ::testing::Test {
protected:
  ros::NodeHandle ros_nh;

  ros::Publisher shutdown_pub;
  image_transport::Publisher image_pub;

  ros::Subscriber pose_sub;
  ros::Subscriber if_sub;
  ros::Subscriber bf_sub;

  awesomo_msgs::AprilTagPose pose_msg;
  geometry_msgs::Vector3 inertial_msg;
  geometry_msgs::Vector3 body_msg;

  NodeTest(void) {
    image_transport::ImageTransport it(this->ros_nh);

    // clang-format off
    this->image_pub = it.advertise(IMAGE_TOPIC, 1);
    this->pose_sub = this->ros_nh.subscribe(TARGET_POSE_TOPIC, 1, &NodeTest::poseCallback, this);
    this->if_sub = this->ros_nh.subscribe(TARGET_IF_TOPIC, 1, &NodeTest::inertialCallback, this);
    this->bf_sub = this->ros_nh.subscribe(TARGET_BPF_TOPIC, 1, &NodeTest::bodyPlanarCallback, this);
    // clang-format on

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  virtual void SetUp(void) {
    sensor_msgs::ImageConstPtr msg;
    cv::Mat image;

    // setup image
    image = cv::imread(TEST_IMAGE);
    image.at<double>(0, 0) = 0.0;  // position x
    image.at<double>(0, 1) = 0.0;  // position y
    image.at<double>(0, 2) = 3.0;  // position z

    image.at<double>(0, 3) = 1.0;  // quaternion w
    image.at<double>(0, 4) = 0.0;  // quaternion x
    image.at<double>(0, 5) = 0.0;  // quaternion y
    image.at<double>(0, 6) = 0.0;  // quaternion z

    // publish image
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    this->image_pub.publish(msg);

    // spin and sleep
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  void poseCallback(const awesomo_msgs::AprilTagPose &msg) {
    this->pose_msg = msg;
  }

  void inertialCallback(const geometry_msgs::Vector3 &msg) {
    this->inertial_msg = msg;
  }

  void bodyPlanarCallback(const geometry_msgs::Vector3 &msg) {
    this->body_msg = msg;
  }
};

TEST_F(NodeTest, poseMsg) {
  ASSERT_EQ(1, this->pose_sub.getNumPublishers());

  ASSERT_EQ(0, this->pose_msg.id);
  ASSERT_TRUE(this->pose_msg.detected);
  ASSERT_NEAR(0.0, this->pose_msg.position.x, 0.2);
  ASSERT_NEAR(0.0, this->pose_msg.position.y, 0.2);
  ASSERT_NEAR(3.0, this->pose_msg.position.z, 0.2);
}

TEST_F(NodeTest, inertialMsg) {
  ASSERT_EQ(1, this->if_sub.getNumPublishers());

  ASSERT_NEAR(0.0, this->inertial_msg.x, 0.2);
  ASSERT_NEAR(0.0, this->inertial_msg.y, 0.2);
  ASSERT_NEAR(0.0, this->inertial_msg.z, 0.2);
}

TEST_F(NodeTest, bodyMsg) {
  ASSERT_EQ(1, this->bf_sub.getNumPublishers());

  ASSERT_NEAR(0.0, this->body_msg.x, 0.2);
  ASSERT_NEAR(0.0, this->body_msg.y, 0.2);
  ASSERT_NEAR(-3.0, this->body_msg.z, 0.2);
}

}  // end of awesomo namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, TEST_NODE_NAME);
  return RUN_ALL_TESTS();
}
