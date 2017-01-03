#include "awesomo_ros/camera_node.hpp"

namespace awesomo {

int CameraNode::configure(std::string node_name, int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // camera
  this->ros_nh->getParam("/camera_config_dir", config_path);
  if (this->camera.configure(config_path) != 0) {
    ROS_ERROR("Failed to configure Camera!");
    return -2;
  };
  this->camera.initialize();

  // image transport
  this->registerImagePublisher("camera_image_topic");

  // register loop callback
  this->registerLoopCallback(std::bind(&CameraNode::loopCallback, this));

  this->configured = true;
  return 0;
}

int CameraNode::loopCallback(void) {
  this->publishImage();
  return 0;
}

int CameraNode::publishImage(void) {
  sensor_msgs::ImageConstPtr img_msg;

  // get frame
  this->camera.getFrame(this->image);
  this->camera.showImage(this->image);

  // publish image
  // clang-format off
  img_msg = cv_bridge::CvImage(
    std_msgs::Header(),
    "bgr8",
    this->image
  ).toImageMsg();
  this->img_pub.publish(img_msg);
  // clang-format on

  return 0;
}

}  // end of awesomo namespace


int main(int argc, char **argv) {
  awesomo::CameraNode node;

  if (node.configure(CAMERA_NODE_NAME, CAMERA_NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure CameraNode!");
    return -1;
  }
  node.loop();

  return 0;
}
