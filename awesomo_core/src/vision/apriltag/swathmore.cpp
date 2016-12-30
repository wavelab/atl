#include "awesomo_core/vision/apriltag/swathmore.hpp"


namespace awesomo {

SwathmoreDetector::SwathmoreDetector(void) {
  this->configured = false;

  this->detector = NULL;

  this->tag_configs.clear();
  this->camera_mode = "";
  this->camera_modes.clear();
  this->camera_configs.clear();
  this->imshow = false;
}

SwathmoreDetector::~SwathmoreDetector(void) {
  // delete family_str;
  // delete family;
  // delete params;
}

int SwathmoreDetector::configure(std::string config_file) {
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

  // tag family
  std::string *family_str = new std::string("Tag16h5");
  TagFamily *family = new TagFamily(*family_str);

  // tag params
  TagDetectorParams *params = new TagDetectorParams();
  params->newQuadAlgorithm = true;

  // tag detector
  family->setErrorRecoveryFraction(0.5);
  this->detector = new TagDetector(*family, *params);

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

std::vector<TagPose> SwathmoreDetector::extractTags(cv::Mat &image) {
  TagPose pose;
  cv::Point2d optical_center;
  TagDetectionArray detections;
  std::vector<TagPose> pose_estimates;

  // setup
  optical_center.x = image.cols * 0.5;
  optical_center.y = image.rows * 0.5;

  // extract tags
  this->detector->process(image, optical_center, detections);

  // calculate tag pose
  for (int i = 0; i < detections.size(); i++) {
    if (this->obtainPose(detections[i], pose) == 0) {
      pose_estimates.push_back(pose);
    }

    // only need 1 tag
    break;
  }

  return pose_estimates;
}

int SwathmoreDetector::obtainPose(TagDetection tag, TagPose &tag_pose) {
  cv::Mat R, T;
  CameraConfig *camera_config;
  double fx, fy, tag_size;

  // setup
  camera_config = this->camera_configs[this->camera_mode];
  fx = camera_config->camera_matrix.at<double>(0, 0);
  fy = camera_config->camera_matrix.at<double>(1, 1);
  tag_size = 0.0;

  // get tag size according to tag id
  if (this->tag_configs.find(tag.id) == this->tag_configs.end()) {
    log_err("ERROR! Tag size for [%d] not configured!\n", (int) tag.id);
    return -2;
  } else {
    tag_size = this->tag_configs[tag.id];
  }

  // caculate pose
  CameraUtil::homographyToPoseCV(fx, fy, tag_size, tag.homography, R, T);

  // convert from camera frame to NED frame
  // camera frame:  (z - forward, x - right, y -down)
  // NED frame:     (x - forward, y - right, z -down)
  tag_pose.id = tag.id;
  tag_pose.detected = true;
  tag_pose.position << T.at<double>(2), T.at<double>(0), T.at<double>(1);

  return 0;
}

void SwathmoreDetector::printTag(TagPose tag) {
  std::cout << "id: " << tag.id << " ";
  std::cout << "[";
  std::cout << "x= " << tag.position(0) << " ";
  std::cout << "y= " << tag.position(1) << " ";
  std::cout << "z= " << tag.position(2);
  std::cout << "]";
  std::cout << std::endl;
}

}  // end of awesomo namespace
