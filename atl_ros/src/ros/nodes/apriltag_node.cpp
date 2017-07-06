#include "atl/ros/nodes/apriltag_node.hpp"

namespace atl {

int AprilTagNode::configure(const std::string &node_name, int hz) {
  std::string apriltag_config;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // detector
  ROS_GET_PARAM("/apriltag/config", apriltag_config);
  if (this->detector.configure(apriltag_config) != 0) {
    ROS_ERROR("Failed to configure AprilTag Detector!");
    return -2;
  };

  // subscribers and publishers
  // clang-format off
  this->registerPublisher<atl_msgs::AprilTagPose>(TARGET_POSE_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_IF_POS_TOPIC);
  this->registerPublisher<std_msgs::Float64>(TARGET_IF_YAW_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_BPF_POS_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_BPF_POS_ENCODER_TOPIC);
  this->registerPublisher<std_msgs::Float64>(TARGET_BPF_YAW_TOPIC);
  this->registerImageSubscriber(CAMERA_IMAGE_TOPIC, &AprilTagNode::imageCallback, this);
  this->registerShutdown(SHUTDOWN);
  // clang-format on

  // downward facing camera (gimbal is NWU frame)
  // NWU frame: (x - forward, y - left, z - up)
  this->camera_offset = Pose(0.0, deg2rad(90.0), 0.0, 0.1, 0.0, 0.0);

  return 0;
}

void AprilTagNode::publishTagPoseMsg(TagPose tag) {
  atl_msgs::AprilTagPose msg;
  buildMsg(tag, msg);
  this->ros_pubs[TARGET_POSE_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetInertialPositionMsg(Vec3 gimbal_position,
                                                    Quaternion gimbal_frame,
                                                    Vec3 target_bpf) {
  geometry_msgs::Vector3 msg;
  Vec3 target_if;

  // transform target from body planar to inertial frame
  target_if =
    Gimbal::getTargetInIF(target_bpf, gimbal_position, gimbal_frame);

  // build and publish msg
  buildMsg(target_if, msg);
  this->ros_pubs[TARGET_IF_POS_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetBodyPositionMsg(Vec3 target_bpf) {
  geometry_msgs::Vector3 msg;
  buildMsg(target_bpf, msg);
  this->ros_pubs[TARGET_BPF_POS_TOPIC].publish(msg);
}
void AprilTagNode::publishTargetBodyPositionEncoderMsg(
  Vec3 target_bpf_encoder) {
  geometry_msgs::Vector3 msg;
  buildMsg(target_bpf_encoder, msg);
  this->ros_pubs[TARGET_BPF_POS_ENCODER_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetInertialYawMsg(TagPose tag,
                                               Quaternion gimbal_frame) {
  double yaw_if;
  Vec3 tag_euler, gimbal_frame_euler;
  std_msgs::Float64 msg;

  // convert orientation in quaternion to euler angles
  quat2euler(gimbal_frame, 321, gimbal_frame_euler);
  quat2euler(tag.orientation, 321, tag_euler);

  // build and publish msg
  yaw_if = wrapTo180(rad2deg(gimbal_frame_euler(2) - tag_euler(2)));
  msg.data = deg2rad(yaw_if);
  this->ros_pubs[TARGET_IF_YAW_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetBodyYawMsg(TagPose tag) {
  Vec3 euler;
  std_msgs::Float64 msg;

  // convert orientation in quaternion to euler angles
  quat2euler(tag.orientation, 321, euler);

  // build and publish msg
  msg.data = euler(2);
  this->ros_pubs[TARGET_BPF_YAW_TOPIC].publish(msg);
}

void AprilTagNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  int retval;
  Vec3 target_cf, target_bpf, gimbal_position;
  Vec3 target_bpf_encoder;
  Quaternion gimbal_frame;
  Quaternion gimbal_joint;
  cv_bridge::CvImagePtr image_ptr;
  std::vector<TagPose> tags;

  Quaternion gimbal_joint_bf;
  Quaternion quad_orientation;
  Vec3 quad_position;
  // parse msg
  image_ptr = cv_bridge::toCvCopy(msg);

  // debug
  if (this->debug_mode) {
    cv::imshow("AprilTagNode Image", image_ptr->image);
    cv::waitKey(1);
  }

  // extract gimbal stats from image
  gimbal_position(0) = image_ptr->image.at<double>(0, 0);
  gimbal_position(1) = image_ptr->image.at<double>(0, 1);
  gimbal_position(2) = image_ptr->image.at<double>(0, 2);

  gimbal_frame.w() = image_ptr->image.at<double>(0, 3);
  gimbal_frame.x() = image_ptr->image.at<double>(0, 4);
  gimbal_frame.y() = image_ptr->image.at<double>(0, 5);
  gimbal_frame.z() = image_ptr->image.at<double>(0, 6);

  gimbal_joint.w() = image_ptr->image.at<double>(0, 7);
  gimbal_joint.x() = image_ptr->image.at<double>(0, 8);
  gimbal_joint.y() = image_ptr->image.at<double>(0, 9);
  gimbal_joint.z() = image_ptr->image.at<double>(0, 10);

  gimbal_joint_bf.w() = image_ptr->image.at<double>(0, 11);
  gimbal_joint_bf.x() = image_ptr->image.at<double>(0, 12);
  gimbal_joint_bf.y() = image_ptr->image.at<double>(0, 13);
  gimbal_joint_bf.z() = image_ptr->image.at<double>(0, 14);

  quad_position(0) = image_ptr->image.at<double>(0, 15);
  quad_position(1) = image_ptr->image.at<double>(0, 16);
  quad_position(2) = image_ptr->image.at<double>(0, 17);

  quad_orientation.w() = image_ptr->image.at<double>(0, 18);
  quad_orientation.x() = image_ptr->image.at<double>(0, 19);
  quad_orientation.y() = image_ptr->image.at<double>(0, 20);
  quad_orientation.z() = image_ptr->image.at<double>(0, 21);

  // remove the gimbal states from image
  for (int i = 0; i < 22; i++) {
    image_ptr->image.at<double>(0, i) = 1;
  }

  // detect tags
  retval = this->detector.extractTags(image_ptr->image, tags);
  if (retval == -1) {
    exit(-1);  // dangerous but necessary
  } else if (tags.size() == 0) {
    return;
  }
  // transform tag in camera frame to body planar frame
  target_cf << tags[0].position(0), tags[0].position(1), tags[0].position(2);
  target_bpf =
    Gimbal::getTargetInBPF(this->camera_offset, target_cf, gimbal_joint);

  // Calculate target frame in bpf from encoders
  Vec3 encoder_rpy_bf;
  Vec3 quad_rpy_if;
  Vec3 joint_encoder_rpy_if;
  Quaternion joint_encoder_quat_if;

  quat2euler(gimbal_joint_bf, 321, encoder_rpy_bf);
  quat2euler(quad_orientation, 321, quad_rpy_if);

  joint_encoder_rpy_if(0) = encoder_rpy_bf(0) + quad_rpy_if(0);
  joint_encoder_rpy_if(1) = encoder_rpy_bf(1) + quad_rpy_if(1);
  joint_encoder_rpy_if(2) = 0.0;

  euler2quat(joint_encoder_rpy_if, 321, joint_encoder_quat_if);
  target_bpf_encoder = Gimbal::getTargetInBPF(
    this->camera_offset, target_cf, joint_encoder_quat_if);

  // publish tag pose
  this->publishTagPoseMsg(tags[0]);
  this->publishTargetInertialPositionMsg(
    gimbal_position, gimbal_frame, target_bpf);
  this->publishTargetInertialYawMsg(tags[0], gimbal_frame);
  this->publishTargetBodyPositionMsg(target_bpf);
  this->publishTargetBodyPositionEncoderMsg(target_bpf_encoder);
  this->publishTargetBodyYawMsg(tags[0]);
}

}  // namespace atl

RUN_ROS_NODE(atl::AprilTagNode, NODE_NAME, NODE_RATE);
