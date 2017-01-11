#include <gtest/gtest.h>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/nodes/camera_node.hpp"

namespace awesomo {

// NODE SETTNGS
#define NODE_NAME "camera_test_node"

// SUBSCRIBE TOPICS
#define CAMERA_IMAGE_TOPIC "/awesomo/camera/image"

class CameraNodeTest : public ::testing::Test {
protected:
  ros::NodeHandle ros_nh;
  image_transport::Subscriber image_sub;

  cv::Mat image;

  CameraNodeTest(void) {
    image_transport::ImageTransport it(this->ros_nh);

    // clang-format off
    this->image_sub = it.subscribe(CAMERA_IMAGE_TOPIC, 1, &CameraNodeTest::imageCallback, this);
    // clang-format on

    ros::spinOnce();
    ros::Duration(5.0).sleep();
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(msg);
    this->image = image_ptr->image;

    // cv::imshow("image", this->image);
    // cv::waitKey(10);
  }
};

TEST_F(CameraNodeTest, poseMsg) {
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(640, this->image.cols);
  ASSERT_EQ(480, this->image.rows);
}

}  // end of awesomo namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, NODE_NAME);
  return RUN_ALL_TESTS();
}
