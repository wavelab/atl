#include "atl/ros/nodes/cam_calib_node.hpp"

namespace atl {

int CamCalibNode::configure(const std::string &node_name, int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // calibration dir
  ROS_GET_PARAM("/camera/calib_dir", this->calib_dir);
  remove_dir(this->calib_dir.c_str());
  int retval = mkdir(this->calib_dir.c_str(), ACCESSPERMS);
  if (retval != 0) {
    ROS_ERROR("Failed to create calibration dir!");
    return -4;
  }

  // measurements file
  this->gimbal_frame_file.open(this->calib_dir + "/gimbal_frame.dat");
  this->gimbal_joint_file.open(this->calib_dir + "/gimbal_joint.dat");
  this->gimbal_encoder_file.open(this->calib_dir + "/gimbal_encoder.dat");

  // register publisher and subscribers
  // clang-format off
  this->registerImageSubscriber(CAMERA_IMAGE_TOPIC, &CamCalibNode::imageCallback, this);
  this->registerSubscriber(GIMBAL_FRAME_ORIENTATION_TOPIC, &CamCalibNode::gimbalFrameCallback, this);
  this->registerSubscriber(GIMBAL_JOINT_ORIENTATION_TOPIC, &CamCalibNode::gimbalJointCallback, this);
  this->registerSubscriber(GIMBAL_ENCODER_ORIENTATION_TOPIC, &CamCalibNode::gimbalJointBodyCallback, this);
  this->registerShutdown(SHUTDOWN_TOPIC);
  // clang-format on

  // register loop callback
  this->registerLoopCallback(std::bind(&CamCalibNode::loopCallback, this));

  this->configured = true;
  return 0;
}

void CamCalibNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);

  size_t img_size = image_ptr->image.total() * image_ptr->image.elemSize();
  size_t img_rows = image_ptr->image.rows;
  size_t img_cols = image_ptr->image.cols;
  size_t row_bytes = img_size / img_rows;
  cv::Mat(img_rows, img_cols, CV_8UC1, image_ptr->image.data, row_bytes)
    .copyTo(this->image);
}

void CamCalibNode::gimbalFrameCallback(const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_frame_orientation);
}

void CamCalibNode::gimbalJointCallback(const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_joint_orientation);
}

void CamCalibNode::gimbalJointBodyCallback(
  const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_joint_body_orientation);
}

void CamCalibNode::saveImage(cv::Mat &image, const int image_number) {
  std::string savepath;

  std::string filename = "img_" + std::to_string(image_number) + ".jpg";
  paths_combine(this->calib_dir, filename, savepath);

  ROS_INFO("Saving calibration image [%s]", savepath.c_str());
  cv::imwrite(savepath, image);
}

void CamCalibNode::saveGimbalMeasurements() {
  // convert measurements in quaternion to euler angles
  Vec3 gimbal_frame_euler;
  Vec3 gimbal_joint_euler;
  Vec3 gimbal_encoder_euler;

  quat2euler(this->gimbal_joint_orientation, 321, gimbal_joint_euler);
  quat2euler(this->gimbal_frame_orientation, 321, gimbal_frame_euler);
  quat2euler(this->gimbal_encoder_orientation, 321, gimbal_encoder_euler);

  // write to file
  this->gimbal_frame_file << gimbal_frame_euler(0) << ",";
  this->gimbal_frame_file << gimbal_frame_euler(1) << ",";
  this->gimbal_frame_file << gimbal_frame_euler(2) << std::endl;

  this->gimbal_joint_file << gimbal_joint_euler(0) << ",";
  this->gimbal_joint_file << gimbal_joint_euler(1) << ",";
  this->gimbal_joint_file << gimbal_joint_euler(2) << std::endl;

  this->gimbal_encoder_file << gimbal_encoder_euler(0) << ",";
  this->gimbal_encoder_file << gimbal_encoder_euler(1) << ",";
  this->gimbal_encoder_file << gimbal_encoder_euler(2) << std::endl;
}

int CamCalibNode::loopCallback() {
  // show image
  if (this->image.empty() == false) {
    cv::imshow("Image", this->image);
  }

  // parse keyboard input
  int key = cv::waitKey(1);
  switch (key) {
    // "esc" or "q" key was pressed
    case 113:
    case 27:
      ROS_INFO("Shutting down CamCalibNode!");
      return -1;

    // "enter" key was pressed
    case 13:
      this->saveImage(this->image, this->image_number);
      this->saveGimbalMeasurements();
      this->image_number++;
      break;
  }

  return 0;
}

}  // namespace atl

RUN_ROS_NODE(atl::CamCalibNode, NODE_NAME, NODE_RATE);
