#include "awesomo_core/vision/apriltag/mit.hpp"


namespace awesomo {

MITDetector::MITDetector(void) {
  this->configured = false;

  this->detector = NULL;
  this->prev_tag = TagPose();
  this->prev_tag_image_width = 0;
  this->prev_tag_image_height = 0;

  this->tag_configs.clear();
  this->camera_mode = "";
  this->camera_modes.clear();
  this->camera_configs.clear();
  this->imshow = false;
}

int MITDetector::configure(std::string config_file) {
  Camera camera;
  ConfigParser parser;
  std::vector<int> tag_ids;
  std::vector<float> tag_sizes;
  std::string config_dir, camera_config;

  // load config
  parser.addParam<std::vector<int>>("tag_ids", &tag_ids);
  parser.addParam<std::vector<float>>("tag_sizes", &tag_sizes);
  parser.addParam<std::string>("camera_config", &camera_config);
  parser.addParam<bool>("imshow", &this->imshow);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // tag detector
  this->detector = new AprilTags::TagDetector(AprilTags::tagCodes16h5);

  // tag configs
  for (int i = 0; i < tag_ids.size(); i++) {
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

int MITDetector::extractTags(cv::Mat &image, std::vector<TagPose> &tags) {
  int retval;
  TagPose pose;
  cv::Mat masked, image_gray;
  std::vector<AprilTags::TagDetection> detections;

  // change mode based on image size
  this->changeMode(image);

  // mask image if tag was last detected
  if (this->prev_tag.detected) {
    retval = this->maskImage(this->prev_tag, image, masked);
    switch (retval) {
      case 0: cv::cvtColor(masked, image_gray, cv::COLOR_BGR2GRAY); break;
      case -4: return -1;
    }
  }
  if (image_gray.empty()) {
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  }

  // extract tags
  detections = this->detector->extractTags(image_gray);

  // calculate tag pose
  for (int i = 0; i < detections.size(); i++) {
    if (this->obtainPose(detections[i], pose) == 0) {
      tags.push_back(pose);

      // keep track of last tag
      this->prev_tag = pose;
      this->prev_tag_image_width = image.cols;
      this->prev_tag_image_height = image.rows;

      // only need 1 tag
      break;
    }
  }

  return 0;
}

int MITDetector::obtainPose(AprilTags::TagDetection tag, TagPose &tag_pose) {
  Mat4 transform;
  Vec3 t;
  Mat3 R;
  CameraConfig camera_config;
  double fx, fy, cx, cy, tag_size;

  // setup
  camera_config = this->camera_configs[this->camera_mode];
  fx = camera_config.camera_matrix.at<double>(0, 0);
  fy = camera_config.camera_matrix.at<double>(1, 1);
  cx = camera_config.camera_matrix.at<double>(0, 2);
  cy = camera_config.camera_matrix.at<double>(1, 2);
  tag_size = 0.0;

  // get tag size according to tag id
  if (this->tag_configs.find(tag.id) == this->tag_configs.end()) {
    log_err("ERROR! Tag size for [%d] not configured!\n", (int) tag.id);
    return -2;
  } else {
    tag_size = this->tag_configs[tag.id];
  }

  // recovering the relative transform of a tag:
  transform = tag.getRelativeTransform(tag_size, fx, fy, cx, cy);
  t = transform.col(3).head(3);
  R = transform.block(0, 0, 3, 3);

  // tag is in camera frame
  // camera frame:  (z - forward, x - right, y - down)
  tag_pose.id = tag.id;
  tag_pose.detected = true;
  tag_pose.position = t;
  tag_pose.orientation = Quaternion(R);

  return 0;
}

int MITDetector::changeMode(cv::Mat &image) {
  int image_width;
  int image_height;
  bool widths_equal;
  bool heights_equal;
  CameraConfig config;

  // setup
  image_width = image.cols;
  image_height = image.rows;

  // traverse all camera modes and change mode based on image size
  for (int i = 0; i < this->camera_modes.size(); i++) {
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

int MITDetector::maskImage(TagPose tag_pose,
                           cv::Mat &image,
                           cv::Mat &masked,
                           double padding) {
  std::string camera_mode;
  int image_width, image_height;
  double x, y, z;
  double fx, fy, px, py;
  double tag_size;
  Vec2 top_left, bottom_right;
  cv::Point p1, p2;
  cv::Mat mask;

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
  if (p1.x > image.cols || p2.x > image.cols) {
    return -3;
  } else if (p1.x > image.cols || p2.x > image.cols) {
    return -3;
  }

  // create mask
  mask = cv::Mat::zeros(image_height, image_width, CV_8U);
  cv::rectangle(mask, p1, p2, cv::Scalar(255, 255, 255), CV_FILLED);

  // mask image
  image.copyTo(masked, mask);
  this->prev_tag.detected = false;

  return 0;
}

void MITDetector::printTag(TagPose tag) {
  std::cout << "id: " << tag.id << " ";
  std::cout << "[";
  std::cout << "x= " << tag.position(0) << " ";
  std::cout << "y= " << tag.position(1) << " ";
  std::cout << "z= " << tag.position(2);
  std::cout << "]";
  std::cout << std::endl;
}

}  // end of awesomo namespace
