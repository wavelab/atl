#include "atl/ros/nodes/apriltag_node.hpp"

namespace atl {

int AprilTagNode::configure(const int hz) {
  std::string apriltag_config;

  // ros node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  // detector
  ROS_GET_PARAM(this->node_name + "/config", apriltag_config);
  if (this->detector.configure(apriltag_config) != 0) {
    ROS_ERROR("Failed to configure AprilTag Detector!");
    return -2;
  };

  // subscribers and publishers
  // clang-format off
  this->addPublisher<atl_msgs::AprilTagPose>(TARGET_POSE_TOPIC);
  this->addPublisher<geometry_msgs::Vector3>(TARGET_W_POS_TOPIC);
  this->addPublisher<std_msgs::Float64>(TARGET_W_YAW_TOPIC);
  this->addPublisher<geometry_msgs::Vector3>(TARGET_P_POS_TOPIC);
  this->addPublisher<geometry_msgs::Vector3>(TARGET_P_POS_ENCODER_TOPIC);
  this->addPublisher<std_msgs::Float64>(TARGET_P_YAW_TOPIC);
  this->addImageSubscriber(CAMERA_IMAGE_TOPIC, &AprilTagNode::imageCallback, this);
  this->addShutdownListener(SHUTDOWN);
  // clang-format on

  // downward facing camera (gimbal is NWU frame)
  // NWU frame: (x - forward, y - left, z - up)
  this->camera_offset =
      Pose(BODY_FRAME, 0.0, deg2rad(90.0), 0.0, 0.1, 0.0, 0.0);

  return 0;
}

void AprilTagNode::publishTagPoseMsg(const TagPose &tag) {
  atl_msgs::AprilTagPose msg;
  buildMsg(tag, msg);
  this->ros_pubs[TARGET_POSE_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetInertialPositionMsg(
    const Vec3 &gimbal_position,
    const Quaternion &gimbal_frame,
    const Vec3 &target_P) {
  geometry_msgs::Vector3 msg;
  Vec3 target_W;

  // transform target from body planar to inertial frame
  target_W = Gimbal::getTargetInIF(target_P, gimbal_position, gimbal_frame);

  // build and publish msg
  buildMsg(target_W, msg);
  this->ros_pubs[TARGET_W_POS_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetBodyPositionMsg(const Vec3 &target_P) {
  geometry_msgs::Vector3 msg;
  buildMsg(target_P, msg);
  this->ros_pubs[TARGET_P_POS_TOPIC].publish(msg);
}
void AprilTagNode::publishTargetBodyPositionEncoderMsg(
    const Vec3 &target_P_encoder) {
  geometry_msgs::Vector3 msg;
  buildMsg(target_P_encoder, msg);
  this->ros_pubs[TARGET_P_POS_ENCODER_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetInertialYawMsg(const TagPose &tag,
                                               const Quaternion &gimbal_frame) {
  // convert orientation in quaternion to euler angles
  const Vec3 gimbal_frame_euler = quatToEuler321(gimbal_frame);
  const Vec3 tag_euler = quatToEuler321(tag.orientation);

  // build and publish msg
  const double yaw_W = wrapTo180(rad2deg(gimbal_frame_euler(2) - tag_euler(2)));

  std_msgs::Float64 msg;
  msg.data = deg2rad(yaw_W);
  this->ros_pubs[TARGET_W_YAW_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetBodyYawMsg(const TagPose &tag) {
  // convert orientation in quaternion to euler angles
  const Vec3 euler = quatToEuler321(tag.orientation);

  // build and publish msg
  std_msgs::Float64 msg;
  msg.data = euler(2);
  this->ros_pubs[TARGET_P_YAW_TOPIC].publish(msg);
}

void AprilTagNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  // parse msg
  const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);

  // debug
  if (this->debug_mode) {
    cv::imshow("AprilTagNode Image", image_ptr->image);
    cv::waitKey(1);
  }

  // extract gimbal stats from image
  const Vec3 gimbal_position{image_ptr->image.at<double>(0, 0),
                             image_ptr->image.at<double>(0, 1),
                             image_ptr->image.at<double>(0, 2)};

  const Quaternion gimbal_frame{image_ptr->image.at<double>(0, 3),
                                image_ptr->image.at<double>(0, 4),
                                image_ptr->image.at<double>(0, 5),
                                image_ptr->image.at<double>(0, 6)};

  const Quaternion gimbal_joint{image_ptr->image.at<double>(0, 7),
                                image_ptr->image.at<double>(0, 8),
                                image_ptr->image.at<double>(0, 9),
                                image_ptr->image.at<double>(0, 10)};

  const Quaternion gimbal_joint_B{image_ptr->image.at<double>(0, 11),
                                  image_ptr->image.at<double>(0, 12),
                                  image_ptr->image.at<double>(0, 13),
                                  image_ptr->image.at<double>(0, 14)};

  const Vec3 quad_position{image_ptr->image.at<double>(0, 15),
                           image_ptr->image.at<double>(0, 16),
                           image_ptr->image.at<double>(0, 17)};

  const Quaternion quad_orientation{image_ptr->image.at<double>(0, 18),
                                    image_ptr->image.at<double>(0, 19),
                                    image_ptr->image.at<double>(0, 20),
                                    image_ptr->image.at<double>(0, 21)};

  // remove the gimbal states from image
  for (int i = 0; i < 22; i++) {
    image_ptr->image.at<double>(0, i) = 1;
  }

  // detect tags
  std::vector<TagPose> tags;
  int retval = this->detector.extractTags(image_ptr->image, tags);
  if (retval == -1) {
    exit(-1); // dangerous but necessary
  } else if (tags.size() == 0) {
    return;
  }

  // transform tag in camera frame to body planar frame
  const Vec3 target_C{tags[0].position(0),
                      tags[0].position(1),
                      tags[0].position(2)};
  const Vec3 target_P =
      Gimbal::getTargetInBPF(this->camera_offset, target_C, gimbal_joint);

  // Calculate target frame in bpf from encoders
  const Vec3 encoder_rpy_B = quatToEuler321(gimbal_joint_B);
  const Vec3 quad_rpy_W = quatToEuler321(quad_orientation);
  const Vec3 joint_encoder_rpy_W{encoder_rpy_B(0) + quad_rpy_W(0),
                                 encoder_rpy_B(1) + quad_rpy_W(1),
                                 0.0};

  Quaternion joint_encoder_quat_W = euler321ToQuat(joint_encoder_rpy_W);
  const Vec3 target_P_encoder = Gimbal::getTargetInBPF(this->camera_offset,
                                                       target_C,
                                                       joint_encoder_quat_W);

  // publish tag pose
  this->publishTagPoseMsg(tags[0]);
  this->publishTargetInertialPositionMsg(gimbal_position,
                                         gimbal_frame,
                                         target_P);
  this->publishTargetInertialYawMsg(tags[0], gimbal_frame);
  this->publishTargetBodyPositionMsg(target_P);
  this->publishTargetBodyPositionEncoderMsg(target_P_encoder);
  this->publishTargetBodyYawMsg(tags[0]);
}

} // namespace atl

RUN_ROS_NODE(atl::AprilTagNode, NODE_RATE);
