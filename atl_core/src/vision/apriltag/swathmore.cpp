#include "atl/vision/apriltag/swathmore.hpp"


namespace atl {

SwathmoreDetector::SwathmoreDetector(void) {
  this->detector = NULL;
}

int SwathmoreDetector::configure(std::string config_file) {
  if (BaseDetector::configure(config_file) != 0) {
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

  return 0;
}

int SwathmoreDetector::extractTags(cv::Mat &image,
                                   std::vector<TagPose> &tags) {
  int retval;
  TagPose pose;
  cv::Mat image_gray;
  cv::Point2d optical_center;
  TagDetectionArray detections;
  CameraConfig camera_config;

  // setup
  camera_config = this->camera_configs[this->camera_mode];
  optical_center.x = camera_config.camera_matrix.at<double>(0, 2);
  optical_center.y = camera_config.camera_matrix.at<double>(1, 2);

  // change mode based on image size
  this->changeMode(image);

  // tranform illumination invariant tag
  if (this->illum_invar) {
    this->illuminationInvariantTransform(image);
  }

  // mask image if tag was last detected
  if (this->prev_tag.detected && this->windowing) {
    retval = this->maskImage(this->prev_tag, image, this->window_padding);
    if (retval == -4) {
      return -1;
    }
  }

  // convert image to gray-scale
  if (image.channels() == 3) {
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  } else {
    image_gray = image;
  }

  // extract tags
  this->detector->process(image_gray, optical_center, detections);

  // calculate tag pose
  for (size_t i = 0; i < detections.size(); i++) {
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

  // imshow
  if (this->imshow) {
    cv::imshow("SwathmoreDetector", image_gray);
    cv::waitKey(1);
  }

  return 0;
}

int SwathmoreDetector::obtainPose(TagDetection tag, TagPose &tag_pose) {
  Vec3 t;
  Mat3 R;
  cv::Mat cv_R, cv_T;
  CameraConfig camera_config;
  double fx, fy, tag_size;

  // setup
  camera_config = this->camera_configs[this->camera_mode];
  fx = camera_config.camera_matrix.at<double>(0, 0);
  fy = camera_config.camera_matrix.at<double>(1, 1);
  tag_size = 0.0;

  // get tag size according to tag id
  if (this->tag_configs.find(tag.id) == this->tag_configs.end()) {
    LOG_ERROR("ERROR! Tag size for [%d] not configured!\n", (int) tag.id);
    return -2;
  } else {
    tag_size = this->tag_configs[tag.id];
  }

  // caculate pose
  CameraUtil::homographyToPoseCV(
    fx, fy, tag_size, tag.homography, cv_R, cv_T);

  // sanity check - calculate euclidean distance between prev and current tag
  // clang-format on
  t << cv_T.at<double>(0), cv_T.at<double>(1), cv_T.at<double>(2);
  R << cv_R.at<double>(0, 0), cv_R.at<double>(0, 1), cv_R.at<double>(0, 2),
    cv_R.at<double>(1, 0), cv_R.at<double>(1, 1), cv_R.at<double>(1, 2),
    cv_R.at<double>(2, 0), cv_R.at<double>(2, 1), cv_R.at<double>(2, 2);
  // clang-format off
  if ((t - this->prev_tag.position).norm() > this->tag_sanity_check) {
    return -1;
  }

  // tag is in camera frame
  // camera frame:  (z - forward, x - right, y - down)
  tag_pose.id = tag.id;
  tag_pose.detected = true;
  tag_pose.position = t;
  tag_pose.orientation = Quaternion(R);

  return 0;
}

}  // namespace atl
