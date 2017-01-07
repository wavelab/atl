#include "awesomo_ros/nodes/apriltag_node.hpp"

namespace awesomo {

int AprilTagNode::configure(const std::string &node_name, int hz) {
  std::string apriltag_config;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // detector
  this->ros_nh->getParam("/apriltag_config", apriltag_config);
  if (this->detector.configure(apriltag_config) != 0) {
    ROS_ERROR("Failed to configure AprilTag Detector!");
    return -2;
  };

  // subscribers and publishers
  // clang-format off
  this->registerPublisher<awesomo_msgs::AprilTagPose>(APRILTAG_POSE_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(GIMBAL_TRACK_TOPIC);
  this->registerImageSubscriber(CAMERA_POSE_TOPIC, &AprilTagNode::imageCallback, this);
  this->registerShutdown(SHUTDOWN);
  // clang-format on

  return 0;
}

void AprilTagNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr image_ptr;
  std::vector<TagPose> tags;
  geometry_msgs::Vector3 track_msg;
  awesomo_msgs::AprilTagPose pose_msg;

  // detect tags
  image_ptr = cv_bridge::toCvCopy(msg);
  tags = this->detector.extractTags(image_ptr->image);

  // debug
  if (this->debug_mode) {
    cv::imshow("AprilTagNode Image", image_ptr->image);
    cv::waitKey(1);
  }

  // publish tag pose
  if (tags.size()) {
    buildAprilTagPoseMsg(tags[0], pose_msg);
    buildAprilTagTrackMsg(tags[0], track_msg);
    this->ros_pubs[GIMBAL_TRACK_TOPIC].publish(track_msg);
  }
  this->ros_pubs[APRILTAG_POSE_TOPIC].publish(pose_msg);
}

}  // end of awesomo namespace

int main(void) {
  awesomo::AprilTagNode node;

  if (node.configure(APRILTAG_NODE_NAME, APRILTAG_NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure AprilTag Node!");
    return -1;
  }
  node.loop();

  return 0;
}
