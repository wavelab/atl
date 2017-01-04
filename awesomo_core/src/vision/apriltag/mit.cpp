#include "awesomo_core/vision/apriltag/mit.hpp"


namespace awesomo {

MITDetector::MITDetector(void) {
  this->configured = false;

  this->detector = NULL;

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

std::vector<TagPose> MITDetector::extractTags(cv::Mat &image) {
  TagPose pose;
  cv::Mat image_gray;
  std::vector<AprilTags::TagDetection> detections;
  std::vector<TagPose> pose_estimates;

  // extract tags
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  detections = this->detector->extractTags(image_gray);

  // calculate tag pose
  for (int i = 0; i < detections.size(); i++) {
    if (this->obtainPose(detections[i], pose) == 0) {
      pose_estimates.push_back(pose);

      // only need 1 tag
      break;
    }
  }

  return pose_estimates;
}

int MITDetector::obtainPose(AprilTags::TagDetection tag, TagPose &tag_pose) {
  Mat4 transform;
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

  // tag is in camera frame
  // camera frame:  (z - forward, x - right, y - down)
  tag_pose.id = tag.id;
  tag_pose.detected = true;
  tag_pose.position = transform.col(3).head(3);

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
