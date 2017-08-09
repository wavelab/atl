#include "atl/ros/nodes/cam_calib_node.hpp"

namespace atl {

int CamCalibNode::configure(const std::string &node_name, int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // get ros params
  ROS_GET_PARAM(this->ros_node_name + "/calib_dir", this->calib_dir);
  ROS_GET_PARAM(this->ros_node_name + "/nb_cameras", this->nb_cameras);
  ROS_GET_PARAM(
    this->ros_node_name + "/camera_1_topic", this->camera_1_topic);
  if (this->nb_cameras == 2) {
    ROS_GET_PARAM(
      this->ros_node_name + "/camera_2_topic", this->camera_2_topic);
  } else if (this->nb_cameras <= 0 || this->nb_cameras > 2) {
    ROS_ERROR("This node only supports maximum of 2 cameras!");
    return -2;
  }

  // calibration dir
  remove_dir(this->calib_dir.c_str());
  int retval = mkdir(this->calib_dir.c_str(), ACCESSPERMS);
  if (retval != 0) {
    ROS_ERROR("Failed to create calibration dir!");
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
  this->gimbal_frame_file.open(this->calib_dir + "/gimbal_frame.dat");
  this->gimbal_joint_file.open(this->calib_dir + "/gimbal_joint.dat");
  this->gimbal_encoder_file.open(this->calib_dir + "/gimbal_encoder.dat");

  // register publisher and subscribers
  // clang-format off
  this->registerImageSubscriber(this->camera_1_topic, &CamCalibNode::image1Callback, this);
  if (this->nb_cameras == 2) {
    this->registerImageSubscriber(this->camera_2_topic, &CamCalibNode::image2Callback, this);
  }
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

void CamCalibNode::saveImages() {
  std::string savepath;
  std::string filename = "img_" + std::to_string(this->image_number) + ".jpg";

  // save image
  ROS_INFO("Saving calibration image [%s]", savepath.c_str());
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
  Vec3 gimbal_frame_euler;
  Vec3 gimbal_joint_euler;
  Vec3 gimbal_encoder_euler;

  quat2euler(this->gimbal_frame_orientation, 321, gimbal_frame_euler);
  quat2euler(this->gimbal_joint_orientation, 321, gimbal_joint_euler);
  quat2euler(this->gimbal_joint_body_orientation, 321, gimbal_encoder_euler);

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
  if (this->image_1.empty() == false) {
    cv::imshow("Camera 1", this->image_1);
  }
  if (this->image_2.empty() == false) {
    cv::imshow("Camera 2", this->image_2);
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
      // save image
      this->saveImages();
      this->saveGimbalMeasurements();
      break;
  }

  return 0;
}

}  // namespace atl

RUN_ROS_NODE(atl::CamCalibNode, NODE_NAME, NODE_RATE);
