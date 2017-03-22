#include "awesomo_ros/nodes/apriltag_node.hpp"

namespace awesomo {

int AprilTagNode::configure(const std::string &node_name, int hz) {
  std::string apriltag_config;
  this->use_mocap = false;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // detector
  ROS_GET_PARAM("/apriltag_config", apriltag_config);
  if (this->detector.configure(apriltag_config) != 0) {
    ROS_ERROR("Failed to configure AprilTag Detector!");
    return -2;
  };

  // Use embedded imu data for a rostopic
  ROS_GET_PARAM("/use_mocap", this->use_mocap);

  // subscribers and publishers
  // clang-format off
  this->registerPublisher<awesomo_msgs::AprilTagPose>(TARGET_POSE_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_IF_POS_TOPIC);
  this->registerPublisher<std_msgs::Float64>(TARGET_IF_YAW_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_BPF_POS_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(TARGET_BPF_POS_ENCODER_TOPIC);
  this->registerPublisher<std_msgs::Float64>(TARGET_BPF_YAW_TOPIC);
  this->registerImageSubscriber(CAMERA_IMAGE_TOPIC, &AprilTagNode::imageCallback, this);
  if (this->use_mocap) {
    this->registerSubscriber(CAMERA_MOCAP_POSE_TOPIC, &AprilTagNode::mocapCallback, this);
    this->registerPublisher<geometry_msgs::Vector3>(TARGET_APR_GROUND_TRUTH);
    this->registerPublisher<geometry_msgs::Vector3>(TARGET_CAM_GROUND_TRUTH);
  }
  this->registerShutdown(SHUTDOWN);
  // clang-format on

  // downward facing camera (gimbal is NWU frame)
  // NWU frame: (x - forward, y - left, z - up)
  this->camera_offset = Pose(0.0, deg2rad(90.0), 0.0, 0.1, 0.0, 0.0);

  return 0;
}

void AprilTagNode::publishTagPoseMsg(TagPose tag) {
  awesomo_msgs::AprilTagPose msg;
  buildMsg(tag, msg);
  this->ros_pubs[TARGET_POSE_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetInertialPositionMsg(Vec3 gimbal_position,
                                                    Quaternion gimbal_frame,
                                                    Vec3 target_bpf) {
  geometry_msgs::Vector3 msg;
  Vec3 target_if;

  // transform target from body planar to inertial frame
  target_if = Gimbal::getTargetInIF(target_bpf,
                                    gimbal_position,
                                    gimbal_frame);

  std::cout << "\n \n target_bpf: \t" << target_bpf.transpose() << std::endl;
  std::cout << "\n gimbal_position: \t" << gimbal_position.transpose()<< std::endl;
  std::cout << "\n target_if_position: \t" << target_if.transpose()<< std::endl;
  // build and publish msg
  buildMsg(target_if, msg);
  this->ros_pubs[TARGET_IF_POS_TOPIC].publish(msg);
}

void AprilTagNode::publishTargetBodyPositionMsg(Vec3 target_bpf) {
  geometry_msgs::Vector3 msg;
  buildMsg(target_bpf, msg);
  this->ros_pubs[TARGET_BPF_POS_TOPIC].publish(msg);
}
void AprilTagNode::publishTargetBodyPositionEncoderMsg(Vec3 target_bpf_encoder) {
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
  cv_bridge::CvImagePtr image_ptr;
  std::vector<TagPose> tags;
  Vec3 target_cf, target_bpf, target_bpf_encoder;
  Quaternion gimbal_joint_bf;

  // parse msg
  image_ptr = cv_bridge::toCvCopy(msg);

  // debug
  if (this->debug_mode) {
    cv::imshow("AprilTagNode Image", image_ptr->image);
    cv::waitKey(1);
  }

  if (this->use_mocap == false) {
    // extract gimbal stats from image
    this->gimbal_position(0) = image_ptr->image.at<double>(0, 0);
    this->gimbal_position(1) = image_ptr->image.at<double>(0, 1);
    this->gimbal_position(2) = image_ptr->image.at<double>(0, 2);

    this->gimbal_frame.w() = image_ptr->image.at<double>(0, 3);
    this->gimbal_frame.x() = image_ptr->image.at<double>(0, 4);
    this->gimbal_frame.y() = image_ptr->image.at<double>(0, 5);
    this->gimbal_frame.z() = image_ptr->image.at<double>(0, 6);

    this->gimbal_joint.w() = image_ptr->image.at<double>(0, 7);
    this->gimbal_joint.x() = image_ptr->image.at<double>(0, 8);
    this->gimbal_joint.y() = image_ptr->image.at<double>(0, 9);
    this->gimbal_joint.z() = image_ptr->image.at<double>(0, 10);

    this->gimbal_joint_bf.w() = image_ptr->image.at<double>(0, 11);
    this->gimbal_joint_bf.x() = image_ptr->image.at<double>(0, 12);
    this->gimbal_joint_bf.y() = image_ptr->image.at<double>(0, 13);
    this->gimbal_joint_bf.z() = image_ptr->image.at<double>(0, 14);

    this->quad_position(0) = image_ptr->image.at<double>(0, 15);
    this->quad_position(1) = image_ptr->image.at<double>(0, 16);
    this->quad_position(2) = image_ptr->image.at<double>(0, 17);

    this->quad_orientation.w() = image_ptr->image.at<double>(0, 18);
    this->quad_orientation.x() = image_ptr->image.at<double>(0, 19);
    this->quad_orientation.y() = image_ptr->image.at<double>(0, 20);
    this->quad_orientation.z() = image_ptr->image.at<double>(0, 21);

    // remove the gimbal states from image
    for (int i = 0; i < 22; i++) {
      image_ptr->image.at<double>(0, i) = 1;
    }
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
  target_bpf = Gimbal::getTargetInBPF(this->camera_offset,
                                      target_cf,
                                      this->gimbal_joint);

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
  target_bpf_encoder = Gimbal::getTargetInBPF(this->camera_offset,
                                              target_cf,
                                              joint_encoder_quat_if);

  // publish tag pose
  this->publishTagPoseMsg(tags[0]);
  this->publishTargetInertialPositionMsg(gimbal_position,
                                         gimbal_frame,
                                         target_bpf);
  this->publishTargetInertialYawMsg(tags[0], gimbal_frame);
  this->publishTargetBodyPositionMsg(target_bpf);
  this->publishTargetBodyPositionEncoderMsg(target_bpf_encoder);
  this->publishTargetBodyYawMsg(tags[0]);
}

void AprilTagNode::mocapCallback(const geometry_msgs::PoseStamped &msg) {

  this->camera_offset = Pose(0.0, 0, 0.0, 0.0, 0.0, 0.0);

  this->gimbal_position(0) = msg.pose.position.x;
  this->gimbal_position(1) = msg.pose.position.y;
  this->gimbal_position(2) = msg.pose.position.z;

  // Transform this

  this->gimbal_joint.w() = msg.pose.orientation.w;
  this->gimbal_joint.x() = msg.pose.orientation.x;
  this->gimbal_joint.y() = msg.pose.orientation.y;
  this->gimbal_joint.z() = msg.pose.orientation.z;

  Vec3 target_ground_truth_apr_frame;
  Vec3 target_offset_inertial;
  geometry_msgs::Vector3 msg_out;

  target_offset_inertial << 0, 0, 0;
  AprilTagNode::calcTargetGroundTruth(target_ground_truth_apr_frame, target_offset_inertial);
  buildMsg(target_ground_truth_apr_frame, msg_out);
  this->ros_pubs[TARGET_APR_GROUND_TRUTH].publish(msg_out);

  AprilTagNode::calcCameraGroundTruth(target_ground_truth_apr_frame);
  AprilTagNode::rotateMocapGimbalPosition();
}

int AprilTagNode::calcTargetGroundTruth(Vec3 &ground_truth, Vec3 inertial_position) {
  Mat4 world_to_cam_T;
  Mat3 world_to_cam_R;
  Eigen::Matrix<double, 4, 1> position;

  world_to_cam_T = Eigen::MatrixXd::Identity(4, 4);
  world_to_cam_R = this->gimbal_joint.toRotationMatrix();

  world_to_cam_T.block<3, 3>(0, 0) = world_to_cam_R;
  world_to_cam_T.block<3, 1>(0, 3) = this->gimbal_position.transpose();
  position = world_to_cam_T.inverse() * inertial_position.homogeneous();

  Eigen::Matrix<double, 4, 4> cam_to_apr_T;
  cam_to_apr_T = Eigen::MatrixXd::Identity(4, 4);
  Mat3 R;
  Vec3 rpy;
  rpy << deg2rad(-90), deg2rad(0), deg2rad(-90);
  euler2rot(rpy, 321, R);
  cam_to_apr_T.block<3, 3>(0, 0) = R;
  position = cam_to_apr_T.inverse() * position;
  // position = cam_to_apr_T * position;
  ground_truth = position.block<3, 1>(0, 0);
  return 0;
}

int AprilTagNode::calcCameraGroundTruth(Vec3 &apr_graph_truth) {
  Mat4 apr_to_cam_T;
  Mat3 apr_to_cam_R;
  Vec3 cam_ground_truth;
  Eigen::Matrix<double, 4, 1> cam_gt;
  apr_to_cam_T = Eigen::MatrixXd::Identity(4, 4);
  geometry_msgs::Vector3 msg;

  Vec3 rpy;
  rpy << deg2rad(-90), deg2rad(0), deg2rad(-90);
  euler2rot(rpy, 321, apr_to_cam_R);

  apr_to_cam_T.block<3, 3>(0, 0) = apr_to_cam_R;
  cam_gt = apr_to_cam_T * apr_graph_truth.homogeneous();

  cam_ground_truth = cam_gt.block<3, 1>(0, 0);

  buildMsg(cam_ground_truth, msg);
  this->ros_pubs[TARGET_CAM_GROUND_TRUTH].publish(msg);
}

int AprilTagNode::rotateMocapGimbalPosition() {
  Mat4 Mocap_to_Camera_Frame_T;
  Mat3 Mocap_to_Camera_Frame_R;
  Eigen::Matrix<double, 4, 1> cam_position;

  Mocap_to_Camera_Frame_T = Eigen::MatrixXd::Identity(4, 4);
  Vec3 euler;
  quat2euler(this->gimbal_joint, 321, euler);
  euler(0) = 0.0;
  euler(1) = 0.0;

  euler2rot(euler, 123, Mocap_to_Camera_Frame_R);
  Mocap_to_Camera_Frame_T.block<3, 3>(0, 0) = Mocap_to_Camera_Frame_R;
  cam_position = Mocap_to_Camera_Frame_T * this->gimbal_position.homogeneous();
  // this->gimbal_position = cam_position.block<3, 1>(0, 0);
  return 0;
}


int AprilTagNode::calcBPFGroundTruth(Vec3 &cam_ground_truth) {

  // Mat4 BPF_to_cam_T;
  // Mat3 BPF_to_cam_R;
  // // Vec3 cam_ground_truth;
  // Eigen::Matrix<double, 4, 1> cam_gt;
  // apr_to_cam_T = Eigen::MatrixXd::Identity(4, 4);
  // geometry_msgs::Vector3 msg;

  return 0;

}

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::AprilTagNode, NODE_NAME, NODE_RATE);
