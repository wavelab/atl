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
  this->registerPublisher<awesomo_msgs::AprilTagPose>(TARGET_POSE_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_IF_POS_TOPIC);
  this->registerPublisher<std_msgs::Float64>(TARGET_IF_YAW_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_BPF_POS_TOPIC);
  this->registerPublisher<std_msgs::Float64>(TARGET_BPF_YAW_TOPIC);
  this->registerImageSubscriber(CAMERA_IMAGE_TOPIC, &AprilTagNode::imageCallback, this);
  this->registerShutdown(SHUTDOWN);
  // clang-format on

  // downward facing camera (gimbal is NWU frame)
  // NWU frame: (x - forward, y - left, z - up)
  this->camera_offset = Pose(0.0, deg2rad(90.0), 0.0, 0.0, 0.0, 0.0);

  return 0;
}

void AprilTagNode::publishTagPoseMsg(TagPose tag) {
  awesomo_msgs::AprilTagPose msg;
  buildMsg(tag, msg);
  this->ros_pubs[TARGET_POSE_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetInertialPositionMsg(Vec3 gimbal_position,
                                                    Quaternion gimbal_orientation,
                                                    Vec3 target_bpf) {
  geometry_msgs::Vector3 msg;
  Vec3 target_enu, target_if;

  target_bpf = gimbal_orientation.toRotationMatrix() * target_bpf;

  // convert target body planar frame from NWU to ENU
  nwu2enu(target_bpf, target_enu);

  // transform target from body to inertial frame
  target_if = gimbal_position + target_enu;

  // Vec3 euler;
  // quat2euler(gimbal_orientation, 321, euler);
  // std::cout << "frame: " << euler.transpose() << std::endl;

  // std::cout << "target_if: " << target_if.transpose() << std::endl;
  // target_if = gimbal_orientation.toRotationMatrix() * target_if;
  // std::cout << "target_if: " << target_if.transpose() << std::endl;
  // std::cout << std::endl;


  // build and publish msg
  buildMsg(target_if, msg);
  this->ros_pubs[TARGET_IF_POS_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetInertialYawMsg(TagPose tag, Quaternion body) {
  double yaw_if;
  Vec3 tag_euler, body_euler;
  std_msgs::Float64 msg;

  // convert orientation in quaternion to euler angles
  quat2euler(body, 321, body_euler);
  quat2euler(tag.orientation, 321, tag_euler);

  // calculate inertial yaw
  yaw_if = -tag_euler(2) + body_euler(2);

  // build and publish msg
  msg.data = yaw_if;
  this->ros_pubs[TARGET_BPF_YAW_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetBodyPositionMsg(Vec3 target_bpf) {
  geometry_msgs::Vector3 msg;
  buildMsg(target_bpf, msg);
  this->ros_pubs[TARGET_BPF_POS_TOPIC].publish(msg);
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
  Vec3 target_cf, target_bpf, gimbal_position;
  Quaternion frame, joint;
  cv_bridge::CvImagePtr image_ptr;
  std::vector<TagPose> tags;

  // parse msg
  image_ptr = cv_bridge::toCvCopy(msg);

  // debug
  if (this->debug_mode) {
    cv::imshow("AprilTagNode Image", image_ptr->image);
    cv::waitKey(1);
  }

  // detect tags
  tags = this->detector.extractTags(image_ptr->image);
  if (tags.size() == 0) {
    return;
  }

  // transform tag in camera frame to body planar frame
  // clang-format off
  target_cf << tags[0].position(0),
               tags[0].position(1),
               tags[0].position(2);
  // clang-format on

  gimbal_position(0) = image_ptr->image.at<double>(0, 0);
  gimbal_position(1) = image_ptr->image.at<double>(0, 1);
  gimbal_position(2) = image_ptr->image.at<double>(0, 2);

  frame.w() = image_ptr->image.at<double>(0, 3);
  frame.x() = image_ptr->image.at<double>(0, 4);
  frame.y() = image_ptr->image.at<double>(0, 5);
  frame.z() = image_ptr->image.at<double>(0, 6);

  joint.w() = image_ptr->image.at<double>(0, 7);
  joint.x() = image_ptr->image.at<double>(0, 8);
  joint.y() = image_ptr->image.at<double>(0, 9);
  joint.z() = image_ptr->image.at<double>(0, 10);

  target_bpf = Gimbal::getTargetInBPF(this->camera_offset,
                                      target_cf,
                                      joint);

  // publish tag pose
  this->publishTagPoseMsg(tags[0]);
  this->publishTargetInertialPositionMsg(gimbal_position, frame, target_bpf);
  this->publishTargetInertialYawMsg(tags[0], this->orientation);
  this->publishTargetBodyPositionMsg(target_bpf);
  this->publishTargetBodyYawMsg(tags[0]);
}

void AprilTagNode::poseCallback(const geometry_msgs::PoseStamped &msg) {
  this->orientation = convertMsg(msg.pose.orientation);
}

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::AprilTagNode, NODE_NAME, NODE_RATE);
