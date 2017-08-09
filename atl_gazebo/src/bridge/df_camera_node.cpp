#include "atl/gazebo/bridge/df_camera_node.hpp"

namespace atl {
namespace gazebo_bridge {

int DFCameraNode::configure(const std::string &node_name, int hz) {
  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // clang-format off
  ROSNode::registerImagePublisher(CAMERA_IMAGE_RTOPIC);

  // setup gazebo client
  if (DFCameraGClient::configure() != 0) {
    ROS_ERROR("Failed to configure DFCameraGClient!");
    return -1;
  }

  this->configured = true;
  return 0;
}

void DFCameraNode::imageCallback(ConstImagePtr &msg) {
  cv::Size image_size;
  DFCameraGClient::imageCallback(msg);

  // convert image to grayscale
  cv::cvtColor(this->image, this->image, cv::COLOR_BGR2GRAY);

  // build image msg
  // clang-format off
  sensor_msgs::ImageConstPtr img_msg;
  img_msg = cv_bridge::CvImage(
    std_msgs::Header(),
    "mono8",
    this->image
  ).toImageMsg();
  // clang-format on

  // publish image
  this->img_pubs[CAMERA_IMAGE_RTOPIC].publish(img_msg);

  // debug
  if (this->debug_mode) {
    cv::imshow("DFCameraNode Image", this->image);
    cv::waitKey(1);
  }
}

}  // namespace gazebo_bridge
}  // namespace atl

RUN_ROS_NODE(atl::gazebo_bridge::DFCameraNode, NODE_NAME, NODE_RATE);
