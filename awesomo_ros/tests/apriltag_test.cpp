#include <gtest/gtest.h>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/nodes/apriltag_node.hpp"

namespace awesomo {

// NODE SETTNGS
#define TEST_NODE_NAME "apriltag_test_node"

// PUBLISH TOPICS
#define SHUTDOWN_TOPIC "/awesomo/apriltag/shutdown"
#define IMAGE_TOPIC "/awesomo/camera/image"

// SUBSCRIBE TOPICS
#define APRILTAG_POSE_TOPIC "/awesomo/apriltag/pose"
#define GIMBAL_TRACK_TOPIC "/awesomo/gimbal/track"

// TEST DATA
#define TEST_IMAGE "test_data/image.jpg"

class AprilTagNodeTest : public ::testing::Test {
protected:
  ros::NodeHandle ros_nh;
  awesomo_msgs::AprilTagPose pose_msg;
  geometry_msgs::Vector3 track_msg;

  ros::Publisher shutdown_pub;
  image_transport::Publisher image_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber track_sub;

  AprilTagNodeTest(void) {
    image_transport::ImageTransport it(this->ros_nh);

    // clang-format off
    this->image_pub = it.advertise(IMAGE_TOPIC, 1);
    this->pose_sub = this->ros_nh.subscribe(APRILTAG_POSE_TOPIC, 1, &AprilTagNodeTest::poseCallback, this);
    this->track_sub = this->ros_nh.subscribe(GIMBAL_TRACK_TOPIC, 1, &AprilTagNodeTest::trackCallback, this);
    // clang-format on

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  void poseCallback(const awesomo_msgs::AprilTagPose &msg) {
    this->pose_msg = msg;
  }

  void trackCallback(const geometry_msgs::Vector3 &msg) {
    std::cout << this->track_msg.x << std::endl;
    std::cout << this->track_msg.y << std::endl;
    std::cout << this->track_msg.z << std::endl;
    this->track_msg = msg;
  }
};

TEST_F(AprilTagNodeTest, poseMsg) {
  sensor_msgs::ImageConstPtr msg;
  cv::Mat image;

  // publish image
  image = cv::imread(TEST_IMAGE);
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  this->image_pub.publish(msg);
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  // assert
  ASSERT_EQ(1, this->pose_sub.getNumPublishers());
  ASSERT_EQ(0, this->pose_msg.tag_id);

  ASSERT_TRUE(this->pose_msg.tag_detected);
  ASSERT_NEAR(0.0, this->pose_msg.tag_position.x, 0.2);
  ASSERT_NEAR(0.0, this->pose_msg.tag_position.y, 0.2);
  ASSERT_NEAR(3.0, this->pose_msg.tag_position.z, 0.2);
}

}  // end of awesomo namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, TEST_NODE_NAME);
  return RUN_ALL_TESTS();
}
