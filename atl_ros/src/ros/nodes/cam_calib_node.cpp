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

  ROS_GET_PARAM(this->node_name + "/nb_cameras", this->nb_cameras);
  ROS_GET_PARAM(this->node_name + "/camera_1_topic", this->camera_1_topic);
  if (this->nb_cameras == 2) {
    ROS_GET_PARAM(this->node_name + "/camera_2_topic", this->camera_2_topic);
  } else if (this->nb_cameras <= 0 || this->nb_cameras > 2) {
    ROS_ERROR("This node only supports maximum of 2 cameras!");
    return -2;
  }

  // calibration dir
  int retval = mkdir(this->calib_dir.c_str(), ACCESSPERMS);
  if (retval != 0) {
    ROS_ERROR("Failed to create calibration dir!");
    ROS_ERROR("Destination already exists [%s]!", this->calib_dir.c_str());
    ROS_ERROR("Delete dir and retry!");
    return -4;
  }

  // camera dir
  this->camera_1_dir = this->calib_dir + "/" + "camera_1";
  retval = mkdir(this->camera_1_dir.c_str(), ACCESSPERMS);
  if (retval != 0) {
    ROS_ERROR("Failed to create camera_1 dir!");
    return -4;
  }

  if (this->nb_cameras == 2) {
    this->camera_2_dir = this->calib_dir + "/" + "camera_2";
    retval = mkdir(this->camera_2_dir.c_str(), ACCESSPERMS);
    if (retval != 0) {
      ROS_ERROR("Failed to create camera_2 dir!");
      return -4;
    }
  }

  // measurements file
  this->gimbal_joint_file.open(this->calib_dir + "/gimbal_joint.dat");
  this->gimbal_encoder_file.open(this->calib_dir + "/gimbal_encoder.dat");

  // register publisher and subscribers
  // clang-format off
  this->registerImageSubscriber(this->camera_1_topic, &CamCalibNode::image1Callback, this);
  if (this->nb_cameras == 2) {
    this->registerImageSubscriber(this->camera_2_topic, &CamCalibNode::image2Callback, this);
  }
  this->registerSubscriber(GIMBAL_JOINT_ORIENTATION_TOPIC, &CamCalibNode::gimbalJointCallback, this);
  this->registerSubscriber(GIMBAL_ENCODER_ORIENTATION_TOPIC, &CamCalibNode::gimbalJointBodyCallback, this);
  this->registerShutdown(SHUTDOWN_TOPIC);
  // clang-format on

  // register loop callback
  this->registerLoopCallback(std::bind(&CamCalibNode::loopCallback, this));

  this->configured = true;
  return 0;
}

void CamCalibNode::imageMsgToCvMat(
  const sensor_msgs::ImageConstPtr &msg, cv::Mat &img) {
  cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);

  size_t img_size = image_ptr->image.total() * image_ptr->image.elemSize();
  size_t img_rows = image_ptr->image.rows;
  size_t img_cols = image_ptr->image.cols;
  size_t row_bytes = img_size / img_rows;
  cv::Mat(img_rows, img_cols, CV_8UC3, image_ptr->image.data, row_bytes)
    .copyTo(img);
}

void CamCalibNode::image1Callback(const sensor_msgs::ImageConstPtr &msg) {
  this->imageMsgToCvMat(msg, this->image_1);
}

void CamCalibNode::image2Callback(const sensor_msgs::ImageConstPtr &msg) {
  this->imageMsgToCvMat(msg, this->image_2);
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
  if (this->nb_cameras == 1 && this->image_1.empty()) {
    return false;
  } else if (
    this->nb_cameras == 2 &&
    (this->image_1.empty() || this->image_2.empty())) {
    return false;
  }

  // chessboard detector flags
  int flags = cv::CALIB_CB_ADAPTIVE_THRESH;
  flags += cv::CALIB_CB_NORMALIZE_IMAGE;
  flags += cv::CALIB_CB_FAST_CHECK;

  // check first camera
  this->corners_1.clear();
  this->chessboard_detected[0] = cv::findChessboardCorners(
    this->image_1, this->chessboard_size, this->corners_1, flags);
  if (this->nb_cameras == 1) {
    return this->chessboard_detected[0];
  }

  // check second camera
  this->corners_2.clear();
  this->chessboard_detected[1] = cv::findChessboardCorners(
    this->image_2, this->chessboard_size, this->corners_2, flags);
  if (this->chessboard_detected[0] && this->chessboard_detected[1]) {
    return true;
  } else {
    return false;
  }
}

void CamCalibNode::showImages() {
  // pre-check
  if (this->nb_cameras == 1 && this->image_1.empty()) {
    return;
  } else if (
    this->nb_cameras == 2 &&
    (this->image_1.empty() || this->image_2.empty())) {
    return;
  }

  // show camera image 1
  if (this->chessboard_detected[0]) {
    cv::Mat img_1 = this->image_1.clone();
    cv::drawChessboardCorners(
      img_1, this->chessboard_size, this->corners_1, true);
    cv::imshow("Camera 1", img_1);
  } else {
    cv::imshow("Camera 1", this->image_1);
  }

  // show camera image 2
  if (this->chessboard_detected[1]) {
    cv::Mat img_2 = this->image_2.clone();
    cv::drawChessboardCorners(
      img_2, this->chessboard_size, this->corners_2, true);
    cv::imshow("Camera 2", img_2);
  } else {
    cv::imshow("Camera 2", this->image_2);
  }
}

void CamCalibNode::saveImages() {
  std::string savepath;
  std::string filename = "img_" + std::to_string(this->image_number) + ".jpg";

  // save image
  ROS_INFO("Saving calibration image [%d]", this->image_number);
  if (this->nb_cameras == 1) {
    paths_combine(this->camera_1_dir, filename, savepath);
    cv::imwrite(savepath, this->image_1);

  } else if (this->nb_cameras == 2) {
    paths_combine(this->camera_1_dir, filename, savepath);
    cv::imwrite(savepath, this->image_1);

    paths_combine(this->camera_2_dir, filename, savepath);
    cv::imwrite(savepath, this->image_2);

  } else {
    ROS_ERROR("This node only supports maximum of 2 cameras!");
    exit(-1);
  }

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

}  // namespace atl

RUN_ROS_NODE(atl::CamCalibNode, NODE_RATE);
