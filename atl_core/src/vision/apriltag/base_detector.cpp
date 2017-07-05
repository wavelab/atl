#include "atl/vision/apriltag/base_detector.hpp"


namespace atl {

BaseDetector::BaseDetector(void) {
  this->configured = false;

  this->prev_tag = TagPose();
  this->prev_tag_image_width = 0;
  this->prev_tag_image_height = 0;

  this->tag_configs.clear();
  this->tag_sanity_check = FLT_MAX;
  this->camera_mode = "";
  this->camera_modes.clear();
  this->camera_configs.clear();
  this->windowing = false;
  this->window_padding = FLT_MAX;
  this->illum_invar = false;
  this->imshow = false;
}

int BaseDetector::configure(std::string config_file) {
  Camera camera;
  ConfigParser parser;
  std::vector<int> tag_ids;
  std::vector<float> tag_sizes;
  std::string config_dir, camera_config;

  // load config
  parser.addParam<std::vector<int>>("tag_ids", &tag_ids);
  parser.addParam<std::vector<float>>("tag_sizes", &tag_sizes);
  parser.addParam<double>("tag_sanity_check", &this->tag_sanity_check);
  parser.addParam<std::string>("camera_config", &camera_config);
  parser.addParam<bool>("illum_invar", &this->illum_invar);
  parser.addParam<bool>("windowing", &this->windowing);
  parser.addParam<double>("window_padding", &this->window_padding);
  parser.addParam<bool>("imshow", &this->imshow);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // tag detector
  // TO BE CONFIGURED BY DERIVATIVE DETECTORS

  // tag configs
  for (size_t i = 0; i < tag_ids.size(); i++) {
    this->tag_configs[tag_ids[i]] = tag_sizes[i];
  }

  // parse camera configs
  config_dir = std::string(dirname((char *) config_file.c_str()));
  paths_combine(config_dir, camera_config, camera_config);
  if (camera.configure(camera_config) != 0) {
    return -1;
  }
  this->camera_mode = camera.modes[0];
  this->camera_modes = camera.modes;
  this->camera_configs = camera.configs;

  this->configured = true;
  return 0;
}

int BaseDetector::illuminationInvariantTransform(cv::Mat &image) {
  // the following is adapted from:
  // Illumination Invariant Imaging: Applications in Robust Vision-based
  // Localisation, Mapping and Classification for Autonomous Vehicles
  // Maddern et al (2014)
  cv::Mat log_ch_1, log_ch_2, log_ch_3;
  double lambda_1, lambda_2, lambda_3;
  double alpha;
  std::vector<cv::Mat> channels(3);

  lambda_1 = 420;
  lambda_2 = 530;
  lambda_3 = 640;

  // clang-format off
  alpha = (lambda_1 * lambda_3 - lambda_1 * lambda_2) /
          (lambda_2 * lambda_3 - lambda_1 * lambda_2);
  // clang-format on

  split(image, channels);
  channels[0].convertTo(channels[0], CV_32F);
  channels[1].convertTo(channels[1], CV_32F);
  channels[2].convertTo(channels[2], CV_32F);

  channels[0].row(0).setTo(cv::Scalar(1));
  channels[1].row(0).setTo(cv::Scalar(1));
  channels[2].row(0).setTo(cv::Scalar(1));

  cv::log(channels[0] / 255.0, log_ch_1);
  cv::log(channels[1] / 255.0, log_ch_2);
  cv::log(channels[2] / 255.0, log_ch_3);

  image = 0.5 + log_ch_2 - alpha * log_ch_3 - (1 - alpha) * log_ch_1;
  image.setTo(0, image < 0);
  image.setTo(1, image > 1);
  cv::normalize(image, image, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  // double min, max;
  // min = 0;
  // max = 1;
  // cv::minMaxLoc(image, &min, &max);
  // std::cout << "min: " << min << std::endl;
  // std::cout << "max: " << max << std::endl;
  // image = (image - min) / (max - min) ;

  return 0;
}

int BaseDetector::changeMode(cv::Mat &image) {
  int image_width;
  int image_height;
  bool widths_equal;
  bool heights_equal;
  CameraConfig config;

  // setup
  image_width = image.cols;
  image_height = image.rows;

  // traverse all camera modes and change mode based on image size
  for (size_t i = 0; i < this->camera_modes.size(); i++) {
    config = this->camera_configs[this->camera_modes[i]];
    widths_equal = (config.image_width == image_width);
    heights_equal = (config.image_height == image_height);

    if (widths_equal && heights_equal) {
      this->camera_mode = this->camera_modes[i];
      return 1;
    }
  }

  return 0;
}

int BaseDetector::maskImage(TagPose tag_pose, cv::Mat &image, double padding) {
  std::string camera_mode;
  int image_width, image_height;
  double x, y, z;
  double fx, fy, px, py;
  double tag_size;
  Vec2 top_left, bottom_right;
  cv::Point p1, p2;
  cv::Mat mask, masked;

  // pre-check
  if (image.cols != this->prev_tag_image_width) {
    this->prev_tag.detected = false;
    return -1;
  } else if (image.rows != this->prev_tag_image_height) {
    this->prev_tag.detected = false;
    return -1;
  }

  // position is in camera frame
  // camera frame:  (z - forward, x - right, y - down)
  x = tag_pose.position(0);
  y = tag_pose.position(1);
  z = tag_pose.position(2);

  // get tag size according to tag id
  if (this->tag_configs.find(tag_pose.id) == this->tag_configs.end()) {
    return -2;
  } else {
    tag_size = this->tag_configs[tag_pose.id];
  }

  // camera intrinsics
  camera_mode = this->camera_mode;
  fx = this->camera_configs[camera_mode].camera_matrix.at<double>(0, 0);
  fy = this->camera_configs[camera_mode].camera_matrix.at<double>(1, 1);
  px = this->camera_configs[camera_mode].camera_matrix.at<double>(0, 2);
  py = this->camera_configs[camera_mode].camera_matrix.at<double>(1, 2);
  image_width = this->camera_configs[camera_mode].image_width;
  image_height = this->camera_configs[camera_mode].image_height;

  // check input image dimensions against configuration file's
  if (image_width != image.cols) {
    log_err("config image width does not match input image's!");
    return -4;
  } else if (image_height != image.rows) {
    log_err("config image height does not match input image's!");
    return -4;
  }

  // calculate 2 corners of tag in inertial frame to be used to create mask
  top_left(0) = x - (tag_size / 2.0) - padding;
  top_left(1) = y - (tag_size / 2.0) - padding;
  bottom_right(0) = x + (tag_size / 2.0) + padding;
  bottom_right(1) = y + (tag_size / 2.0) + padding;

  // project back to image frame (what it would look like in image)
  top_left(0) = (fx * top_left(0) / z) + px;
  top_left(1) = (fy * top_left(1) / z) + py;
  bottom_right(0) = (fx * bottom_right(0)/ z) + px;
  bottom_right(1) = (fy * bottom_right(1) / z) + py;

  // create and check mask coordinates
  p1 = cv::Point(top_left(0), top_left(1));
  p2 = cv::Point(bottom_right(0), bottom_right(1));
  p1.x = (p1.x > image.cols) ? image.cols : p1.x;
  p1.y = (p1.y > image.rows) ? image.rows: p1.y;
  p2.x = (p2.x > image.cols) ? image.cols : p2.x;
  p2.y = (p2.y > image.rows) ? image.rows: p2.y;

  // create mask
  mask = cv::Mat::zeros(image_height, image_width, CV_8U);
  cv::rectangle(mask, p1, p2, cv::Scalar(255, 255, 255), CV_FILLED);

  // mask image
  image.copyTo(masked, mask);
  masked.copyTo(image);
  this->prev_tag.detected = false;

  return 0;
}

void BaseDetector::printTag(TagPose tag) {
  std::cout << "id: " << tag.id << " ";
  std::cout << "[";
  std::cout << "x= " << tag.position(0) << " ";
  std::cout << "y= " << tag.position(1) << " ";
  std::cout << "z= " << tag.position(2);
  std::cout << "]";
  std::cout << std::endl;
}

}  // namespace atl
