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
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_IF_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_BPF_TOPIC);
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

void AprilTagNode::publishTargetBodyPositionMsg(Vec3 target_bpf) {
  geometry_msgs::Vector3 msg;
  buildMsg(target_bpf, msg);
  this->ros_pubs[TARGET_BPF_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetInertialPositionMsg(Vec3 gimbal_position,
                                                    Vec3 target_bpf) {
  geometry_msgs::Vector3 msg;
  Vec3 target_enu, target_if;

  // convert target body planar frame from NWU to ENU
  nwu2enu(target_bpf, target_enu);

  // transform target from body to inertial frame
  target_if = gimbal_position + target_enu;

  // build and publish msg
  buildMsg(target_if, msg);
  this->ros_pubs[TARGET_IF_TOPIC].publish(msg);
}

void AprilTagNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  Vec3 target_cf, target_bpf, gimbal_position;
  Quaternion joint_if;
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

  joint_if.w() = image_ptr->image.at<double>(0, 3);
  joint_if.x() = image_ptr->image.at<double>(0, 4);
  joint_if.y() = image_ptr->image.at<double>(0, 5);
  joint_if.z() = image_ptr->image.at<double>(0, 6);

  target_bpf = this->getTargetInBPF(target_cf, joint_if);

  // publish tag pose
  this->publishTagPoseMsg(tags[0]);
  this->publishTargetBodyPositionMsg(target_bpf);
  this->publishTargetInertialPositionMsg(gimbal_position, target_bpf);
}

Vec3 AprilTagNode::getTargetInBF(Vec3 target_cf) {
  Vec3 target_nwu;
  Mat3 R;
  Vec3 t;

  // transform camera frame to NWU frame
  // camera frame:  (x - right, y - down, z - forward)
  // NWU frame:  (x - forward, y - left, z - up)
  target_nwu(0) = target_cf(2);
  target_nwu(1) = -target_cf(0);
  target_nwu(2) = -target_cf(1);

  // camera mount offset
  R = this->camera_offset.rotationMatrix();
  t = this->camera_offset.position;

  // transform target from camera frame to gimbal joint frame
  return (R * target_nwu + t);
}

Vec3 AprilTagNode::getTargetInBPF(Vec3 target_cf, Quaternion joint_if) {
  Vec3 p;
  Mat3 R;
  Vec3 target_bpf;

  // transform target in camera frame to gimbal joint frame
  p = this->getTargetInBF(target_cf);

  // joints are assumed to be NWU frame
  R = joint_if.toRotationMatrix();

  // transform target in gimbal joint frame to gimbal body planar frame
  target_bpf = R * p;

  return target_bpf;
}

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::AprilTagNode, NODE_NAME, NODE_RATE);
