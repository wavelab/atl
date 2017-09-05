#include "atl/ros/nodes/pg_camera_node.hpp"

namespace atl {

int PGCameraNode::configure(const int hz) {
  std::string config_path;
  std::string guid_str;

  // ros node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  // camera
  ROS_GET_PARAM(this->node_name + "/guid", guid_str);
  this->guid = std::stoull(guid_str); // ros param cannot parse a uint64_t
  ROS_GET_PARAM(this->node_name + "/stamp_image", this->stamp_image);
  ROS_GET_PARAM(this->node_name + "/image_topic", this->image_topic);
  ROS_GET_PARAM(this->node_name + "/config_dir", config_path);
  if (this->camera.configure(config_path) != 0) {
    ROS_ERROR("Failed to configure Camera!");
    return -2;
  };
  this->camera.initialize(this->guid);
  // this->camera.initialize();

  // register publisher and subscribers
  // clang-format off
  this->addImagePublisher(this->image_topic);
  if (this->stamp_image) {
    this->addSubscriber(GIMBAL_POSITION_TOPIC, &PGCameraNode::gimbalPositionCallback, this);
    this->addSubscriber(GIMBAL_FRAME_ORIENTATION_TOPIC, &PGCameraNode::gimbalFrameCallback, this);
    this->addSubscriber(GIMBAL_JOINT_ORIENTATION_TOPIC, &PGCameraNode::gimbalJointCallback, this);
    this->addSubscriber(ENCODER_ORIENTATION_TOPIC, &PGCameraNode::gimbalJointBodyCallback, this);
    this->addSubscriber(LT_BODY_POSITION_TOPIC , &PGCameraNode::targetPositionCallback, this);
    this->addSubscriber(LT_DETECTED_TOPIC , &PGCameraNode::targetDetectedCallback, this);

    this->addSubscriber(QUAD_POSITION_TOPIC, &PGCameraNode::quadPositionCallback, this);
    this->addSubscriber(QUAD_ORIENTATION_TOPIC, &PGCameraNode::quadOrientationCallback, this);
  }

  this->addShutdownListener(SHUTDOWN_TOPIC);
  // clang-format on

  // register loop callback
  this->addLoopCallback(std::bind(&PGCameraNode::loopCallback, this));

  this->configured = true;
  return 0;
}

int PGCameraNode::publishImage() {

  // encode position and orientation into image (first 11 pixels in first row)
  if (this->stamp_image) {
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

    this->image.at<double>(0, 11) = this->gimbal_joint_body_orientation.w();
    this->image.at<double>(0, 12) = this->gimbal_joint_body_orientation.x();
    this->image.at<double>(0, 13) = this->gimbal_joint_body_orientation.y();
    this->image.at<double>(0, 14) = this->gimbal_joint_body_orientation.z();

    this->image.at<double>(0, 15) = this->quadrotor_position(0);
    this->image.at<double>(0, 16) = this->quadrotor_position(1);
    this->image.at<double>(0, 17) = this->quadrotor_position(2);

    this->image.at<double>(0, 18) = this->quadrotor_orientation.w();
    this->image.at<double>(0, 19) = this->quadrotor_orientation.x();
    this->image.at<double>(0, 20) = this->quadrotor_orientation.y();
    this->image.at<double>(0, 21) = this->quadrotor_orientation.z();
  }

  // publish image
  std_msgs::Header header;
  header.seq = this->ros_seq;
  header.stamp = ros::Time::now();
  header.frame_id = this->node_name;

  const std::string image_type = this->camera.config.image_type;
  const sensor_msgs::ImageConstPtr img_msg =
      cv_bridge::CvImage(header, image_type, this->image).toImageMsg();
  this->img_pubs[this->image_topic].publish(img_msg);

  return 0;
}

void PGCameraNode::gimbalPositionCallback(const geometry_msgs::Vector3 &msg) {
  convertMsg(msg, this->gimbal_position);
}

void PGCameraNode::gimbalFrameCallback(const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_frame_orientation);
}

void PGCameraNode::gimbalJointCallback(const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_joint_orientation);
}

void PGCameraNode::gimbalJointBodyCallback(
    const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_joint_body_orientation);
}

void PGCameraNode::targetPositionCallback(const geometry_msgs::Vector3 &msg) {
  convertMsg(msg, this->target_pos_bf);
}

void PGCameraNode::targetDetectedCallback(const std_msgs::Bool &msg) {
  convertMsg(msg, this->target_detected);
}

void PGCameraNode::quadPositionCallback(const dji_sdk::LocalPosition &msg) {
  Vec3 pos_ned;
  pos_ned(0) = msg.x;
  pos_ned(1) = msg.y;
  pos_ned(2) = msg.z;
  this->quadrotor_position = ned2nwu(pos_ned);
}

void PGCameraNode::quadOrientationCallback(
    const dji_sdk::AttitudeQuaternion &msg) {
  Quaternion orientation_ned;

  orientation_ned.w() = msg.q0;
  orientation_ned.x() = msg.q1;
  orientation_ned.y() = msg.q2;
  orientation_ned.z() = msg.q3;

  // transform pose position and orientation
  // from NED to ENU and NWU
  this->quadrotor_orientation = ned2nwu(orientation_ned);
}

int PGCameraNode::loopCallback() {
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
    } else {
      this->camera.changeMode("160x120");
    }
  }

  this->camera.getFrame(this->image);
  this->publishImage();

  return 0;
}

} // eof atl namespace

RUN_ROS_NODE(atl::PGCameraNode, NODE_RATE);
