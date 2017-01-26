#include "awesomo_ros/nodes/pg_camera_node.hpp"

namespace awesomo {

int PGCameraNode::configure(std::string node_name, int hz) {
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

  // register publisher and subscribers
  this->registerImagePublisher(CAMERA_IMAGE_TOPIC);
  this->registerSubscriber(GIMBAL_FRAME_ORIENTATION_TOPIC, &PGCameraNode::gimbalFrameCallback, this);
  this->registerSubscriber(GIMBAL_JOINT_ORIENTATION_TOPIC, &PGCameraNode::gimbalJointCallback, this);
  // this->registerShutdown(SHUTDOWN_TOPIC);

  // register loop callback
  this->registerLoopCallback(std::bind(&PGCameraNode::loopCallback, this));

  this->configured = true;
  return 0;
}

int PGCameraNode::publishImage(void) {
  sensor_msgs::ImageConstPtr img_msg;
  cv::Mat grey;
  // cv::cvtColor(image, grey, CV_BGR2GRAY);

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  // clang-format off
  img_msg = cv_bridge::CvImage(
    header,
    "rbg8",
    image
  ).toImageMsg();
  this->img_pub.publish(img_msg);
  // clang-format on

  return 0;
}

void PGCameraNode::gimbalFrameCallback(const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_frame);
}

void PGCameraNode::gimbalJointCallback(const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_joint);
}

int PGCameraNode::loopCallback(void) {
  this->camera.getFrame(this->image);
  // this->camera.showImage(this->image);
  this->publishImage();

  return 0;
}

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::PGCameraNode, NODE_NAME, NODE_RATE);
