#include "awesomo_ros/camera_node.hpp"

namespace awesomo {

CameraNode::CameraNode(void) {
  this->configured = false;
}

int CameraNode::configure(std::string node_name, int hz) {
  std::string config_path;

  // ros node
  ROSNode::configure(node_name, hz);

  // camera
  this->ros_nh->getParam("/config_dir", config_path);
  if (this->camera.configure(config_path) != 0) {
    return -1;
  };
  this->camera.initialize();

  // image transport
  image_transport::ImageTransport it(*this->ros_nh);
  this->img_pub = it.advertise(CAMERA_IMAGE_TOPIC, 10);

  // register loop callback
  this->registerLoopCallback(std::bind(&CameraNode::loopCallback, this));

  this->configured = true;

  return 0;
}

int CameraNode::loopCallback(void) {
  this - publishImage();
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

  // configure and loop
  if (node.configure(NODE_NAME, NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure CameraNode!");
    return -1;
  }
  node.loop();

  return 0;
}
