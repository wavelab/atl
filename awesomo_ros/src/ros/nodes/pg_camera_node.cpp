#include "awesomo_ros/nodes/pg_camera_node.hpp"


namespace awesomo {

int PGCameraNode::configure(std::string node_name, int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // camera
  ROS_GET_PARAM("/camera_config_dir", config_path);
  if (this->camera.configure(config_path) != 0) {
    ROS_ERROR("Failed to configure Camera!");
    return -2;
  };
  this->camera.initialize();

  // register publisher and subscribers
  // clang-format off
  this->registerImagePublisher(CAMERA_IMAGE_TOPIC);
  this->registerSubscriber(GIMBAL_FRAME_ORIENTATION_TOPIC, &PGCameraNode::gimbalFrameCallback, this);
  this->registerSubscriber(GIMBAL_JOINT_ORIENTATION_TOPIC, &PGCameraNode::gimbalJointCallback, this);
  this->registerSubscriber(LT_BODY_POSITION_TOPIC , &PGCameraNode::targetPositionCallback, this);
  this->registerSubscriber(LT_DETECTED_TOPIC , &PGCameraNode::targetDetectedCallback, this);
  this->registerShutdown(SHUTDOWN_TOPIC);
  // clang-format on

  // register loop callback
  this->registerLoopCallback(std::bind(&PGCameraNode::loopCallback, this));

  this->configured = true;
  return 0;
}

int PGCameraNode::publishImage(void) {
  sensor_msgs::ImageConstPtr img_msg;


  // encode position and orientation into image (first 11 pixels in first row)
  // if (this->gimbal_mode) {
  this->image.at<double>(0, 0) = this->gimbal_position(0);
  this->image.at<double>(0, 1) = this->gimbal_position(1);
  this->image.at<double>(0, 2) = this->gimbal_position(2);

  this->image.at<double>(0, 3) = this->gimbal_frame_orientation.w();
  this->image.at<double>(0, 4) = this->gimbal_frame_orientation.x();
  this->image.at<double>(0, 5) = this->gimbal_frame_orientation.y();
  this->image.at<double>(0, 6) = this->gimbal_frame_orientation.z();

  this->image.at<double>(0, 7) = this->gimbal_joint_orientation.w();
  this->image.at<double>(0, 8) = this->gimbal_joint_orientation.x();
  this->image.at<double>(0, 9) = this->gimbal_joint_orientation.y();
  this->image.at<double>(0, 10) = this->gimbal_joint_orientation.z();
  // }

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

void PGCameraNode::gimbalFrameCallback(const geometry_msgs::Quaternion &msg) {
  this->gimbal_frame_orientation.w() = msg.w;
  this->gimbal_frame_orientation.x() = msg.x;
  this->gimbal_frame_orientation.y() = msg.y;
  this->gimbal_frame_orientation.z() = msg.z;
}

void PGCameraNode::gimbalJointCallback(const geometry_msgs::Quaternion &msg) {
  this->gimbal_joint_orientation.w() = msg.w;
  this->gimbal_joint_orientation.x() = msg.x;
  this->gimbal_joint_orientation.y() = msg.y;
  this->gimbal_joint_orientation.z() = msg.z;
}

void PGCameraNode::targetPositionCallback(const geometry_msgs::Vector3 &msg) {
  convertMsg(msg, this->target_pos_bf);
}

void PGCameraNode::targetDetectedCallback(const std_msgs::Bool &msg) {
  convertMsg(msg, this->target_detected);
}

int PGCameraNode::loopCallback(void) {
  double dist;

  // change mode depending on apriltag distance
  if (this->target_detected == false) {
    this->camera.changeMode("640x480");

  } else {
    dist = -1 * this->target_pos_bf(2);
    if (dist > 10.0) {
      this->camera.changeMode("640x480");
    } else if (dist > 0.3) {
      this->camera.changeMode("320x240");
    } else  {
      this->camera.changeMode("160x120");
    }
  }

  // this->camera.showImage(this->image);
  this->camera.getFrame(this->image);
  this->publishImage();

  return 0;
}

} // eof awesomo namespace

RUN_ROS_NODE(awesomo::PGCameraNode, NODE_NAME, NODE_RATE);
