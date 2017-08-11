#include "atl/ros/nodes/cam_calib_node.hpp"

namespace atl {

int CamCalibNode::configure(int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  // get ros params
  ROS_GET_PARAM(this->node_name + "/calib_dir", this->calib_dir);

  int chessboard_rows = 0;
  int chessboard_cols = 0;
  ROS_GET_PARAM(this->node_name + "/chessboard_rows", chessboard_rows);
  ROS_GET_PARAM(this->node_name + "/chessboard_cols", chessboard_cols);
  this->chessboard_size = cv::Size(chessboard_cols, chessboard_rows);

  ROS_GET_PARAM(this->node_name + "/static_camera_topic",
                this->static_camera_topic);
  ROS_GET_PARAM(this->node_name + "/gimbal_camera_topic",
                this->gimbal_camera_topic);

  // calibration dir
  int retval = mkdir(this->calib_dir.c_str(), ACCESSPERMS);
  if (retval != 0) {
    ROS_ERROR("Failed to create calibration dir!");
    ROS_ERROR("Destination already exists [%s]!", this->calib_dir.c_str());
    ROS_ERROR("Delete dir and retry!");
    return -4;
  }

  // camera dirs
  this->static_camera_dir = this->calib_dir + "/" + "static_camera";
  retval = mkdir(this->static_camera_dir.c_str(), ACCESSPERMS);
  if (retval != 0) {
    ROS_ERROR("Failed to create static_camera dir!");
    return -4;
  }

  this->gimbal_camera_dir = this->calib_dir + "/" + "gimbal_camera";
  retval = mkdir(this->gimbal_camera_dir.c_str(), ACCESSPERMS);
  if (retval != 0) {
    ROS_ERROR("Failed to create gimbal_camera dir!");
    return -4;
  }

  // measurements file
  this->gimbal_joint_file.open(this->calib_dir + "/gimbal_joint.dat");
  this->gimbal_encoder_file.open(this->calib_dir + "/gimbal_encoder.dat");

  // register publisher and subscribers
  // clang-format off
  this->registerImageSubscriber(this->static_camera_topic, &CamCalibNode::staticCameraCallback, this);
  this->registerImageSubscriber(this->gimbal_camera_topic, &CamCalibNode::gimbalCameraCallback, this);
  this->registerSubscriber(GIMBAL_JOINT_ORIENTATION_TOPIC, &CamCalibNode::gimbalJointCallback, this);
  this->registerSubscriber(GIMBAL_ENCODER_ORIENTATION_TOPIC, &CamCalibNode::gimbalJointBodyCallback, this);
  this->registerShutdown(SHUTDOWN_TOPIC);
  // clang-format on

  // register loop callback
  this->registerLoopCallback(std::bind(&CamCalibNode::loopCallback, this));

  this->configured = true;
  return 0;
}

void CamCalibNode::imageMsgToCvMat(const sensor_msgs::ImageConstPtr &msg,
                                   cv::Mat &img) {
  cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);

  size_t img_size = image_ptr->image.total() * image_ptr->image.elemSize();
  size_t img_rows = image_ptr->image.rows;
  size_t img_cols = image_ptr->image.cols;
  size_t row_bytes = img_size / img_rows;
  cv::Mat(img_rows, img_cols, CV_8UC3, image_ptr->image.data, row_bytes)
      .copyTo(img);
}

void CamCalibNode::staticCameraCallback(const sensor_msgs::ImageConstPtr &msg) {
  this->imageMsgToCvMat(msg, this->static_camera_image);
}

void CamCalibNode::gimbalCameraCallback(const sensor_msgs::ImageConstPtr &msg) {
  this->imageMsgToCvMat(msg, this->gimbal_camera_image);
}

void CamCalibNode::gimbalJointCallback(const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_joint_orientation);
}

void CamCalibNode::gimbalJointBodyCallback(
    const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_joint_body_orientation);
}

bool CamCalibNode::chessboardDetected() {
  // reset detected flags
  this->chessboard_detected[0] = false;
  this->chessboard_detected[1] = false;

  // pre-check
  if (this->static_camera_image.empty() || this->gimbal_camera_image.empty()) {
    return false;
  }

  // chessboard detector flags
  int flags = cv::CALIB_CB_ADAPTIVE_THRESH;
  flags += cv::CALIB_CB_NORMALIZE_IMAGE;
  flags += cv::CALIB_CB_FAST_CHECK;

  // check first camera
  this->corners_1.clear();
  this->chessboard_detected[0] = cv::findChessboardCorners(
      this->static_camera_image, this->chessboard_size, this->corners_1, flags);

  // check second camera
  this->corners_2.clear();
  this->chessboard_detected[1] = cv::findChessboardCorners(
      this->gimbal_camera_image, this->chessboard_size, this->corners_2, flags);

  // check results
  if (this->chessboard_detected[0] && this->chessboard_detected[1]) {
    return true;
  } else {
    return false;
  }
}

void CamCalibNode::showImages() {
  // pre-check
  if (this->static_camera_image.empty() || this->gimbal_camera_image.empty()) {
    return;
  }

  // show static camera image
  if (this->chessboard_detected[0]) {
    cv::Mat img_1 = this->static_camera_image.clone();
    cv::drawChessboardCorners(
        img_1, this->chessboard_size, this->corners_1, true);
    cv::imshow("Camera 1", img_1);
  } else {
    cv::imshow("Camera 1", this->static_camera_image);
  }

  // show gimbal camera image
  if (this->chessboard_detected[1]) {
    cv::Mat img_2 = this->gimbal_camera_image.clone();
    cv::drawChessboardCorners(
        img_2, this->chessboard_size, this->corners_2, true);
    cv::imshow("Camera 2", img_2);
  } else {
    cv::imshow("Camera 2", this->gimbal_camera_image);
  }
}

void CamCalibNode::saveImages() {
  std::string savepath;
  std::string filename = "img_" + std::to_string(this->image_number) + ".jpg";

  // save image
  ROS_INFO("Saving calibration image [%d]", this->image_number);
  paths_combine(this->static_camera_dir, filename, savepath);
  cv::imwrite(savepath, this->static_camera_image);

  paths_combine(this->gimbal_camera_dir, filename, savepath);
  cv::imwrite(savepath, this->gimbal_camera_image);

  // update image number
  this->image_number++;
}

void CamCalibNode::saveGimbalMeasurements() {
  // convert measurements in quaternion to euler angles
  Vec3 gimbal_joint_euler;
  Vec3 gimbal_encoder_euler;

  quat2euler(this->gimbal_joint_orientation, 321, gimbal_joint_euler);
  quat2euler(this->gimbal_joint_body_orientation, 321, gimbal_encoder_euler);

  // write to file
  this->gimbal_joint_file << gimbal_joint_euler(0) << ",";
  this->gimbal_joint_file << gimbal_joint_euler(1) << ",";
  this->gimbal_joint_file << gimbal_joint_euler(2) << std::endl;

  this->gimbal_encoder_file << gimbal_encoder_euler(0) << ",";
  this->gimbal_encoder_file << gimbal_encoder_euler(1) << ",";
  this->gimbal_encoder_file << gimbal_encoder_euler(2) << std::endl;
}

int CamCalibNode::loopCallback() {
  // detect and show camera images
  bool data_ok = this->chessboardDetected();
  this->showImages();

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
    // save image
    if (data_ok) {
      this->saveImages();
      this->saveGimbalMeasurements();
    } else {
      ROS_WARN("Chessboard not inview of camera(s), NOT SAVING!");
    }
    break;
  }

  return 0;
}

} // namespace atl

RUN_ROS_NODE(atl::CamCalibNode, NODE_RATE);
