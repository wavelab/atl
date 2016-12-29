#include "awesomo_core/vision/apriltag/swathmore.hpp"


namespace awesomo {

SwathmoreDetector::SwathmoreDetector(void) {
  this->configured = false;

  this->detector = NULL;

  this->tag_sizes.clear();
  this->camera_matrix = cv::Mat(1, 1, CV_64F);
  this->imshow = false;
}

SwathmoreDetector::~SwathmoreDetector(void) {
  // delete family_str;
  // delete family;
  // delete params;
}

int SwathmoreDetector::configure(std::string config_file) {
  // tag family
  std::string *family_str = new std::string("Tag16h5");
  TagFamily *family = new TagFamily(*family_str);

  // tag params
  TagDetectorParams *params = new TagDetectorParams();
  params->newQuadAlgorithm = true;

  // tag detector
  family->setErrorRecoveryFraction(0.5);
  this->detector = new TagDetector(*family, *params);

  // detector properties
  this->tag_sizes.clear();
  this->camera_matrix = cv::Mat(1, 1, CV_64F);
  this->imshow = false;

  this->configured = true;
  return 0;
}

int SwathmoreDetector::obtainPose(TagDetection tag, TagPose &tag_pose) {
  cv::Mat R, T;
  double fx, fy, tag_size;

  // setup
  fx = this->camera_matrix.at<double>(0, 0);
  fy = this->camera_matrix.at<double>(1, 1);
  tag_size = 0.0;

  // get tag size according to tag id
  if (this->tag_sizes.find(tag.id) == this->tag_sizes.end()) {
    log_err("ERROR! Tag size for [%d] not configured!\n", (int) tag.id);
    return -2;
  } else {
    tag_size = this->tag_sizes[tag.id];
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
    if (this->tag_sizes.find(detections[i].id) != this->tag_sizes.end()) {
      if (this->obtainPose(detections[i], pose) == 0) {
        pose_estimates.push_back(pose);
      }

      // only need 1 tag
      break;
    }
  }

  return pose_estimates;
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
