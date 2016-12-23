#include "awesomo_core/vision/apriltag.hpp"


namespace awesomo {

Detector::Detector(void) {
  std::string *family_str = new std::string("Tag16h5");
  TagFamily *family = new TagFamily(*family_str);
  TagDetectorParams *params = new TagDetectorParams();
  params->newQuadAlgorithm = true;

  family->setErrorRecoveryFraction(0.5);
  this->detector = new TagDetector(*family, *params);

  this->apriltag_imshow = 0;
  this->use_estimator = false;
  this->estimator_initialized = false;
}

Detector::Detector(int apriltag_imshow) {
  std::string *family_str = new std::string("Tag16h5");
  TagFamily *family = new TagFamily(*family_str);
  TagDetectorParams *params = new TagDetectorParams();
  params->newQuadAlgorithm = true;

  family->setErrorRecoveryFraction(0.5);
  this->detector = new TagDetector(*family, *params);

  this->apriltag_imshow = apriltag_imshow;
  this->use_estimator = false;
  this->estimator_initialized = false;
}

static cv::Rect enlargeROI(cv::Mat &frame,
                           cv::Rect boundingBox,
                           int padding) {
  cv::Rect rect = cv::Rect(boundingBox.x - padding,
                           boundingBox.y - padding,
                           boundingBox.width + (padding * 2.0),
                           boundingBox.height + (padding * 2.0));

  // check the size of the roi
  if (rect.x < 0) {
    rect.x = 0;
  }
  if (rect.y < 0) {
    rect.y = 0;
  }
  if (rect.x + rect.width >= frame.cols) {
    rect.width = frame.cols - rect.x;
  }
  if (rect.y + rect.height >= frame.rows) {
    rect.height = frame.rows - rect.y;
  }

  return rect;
}

void Detector::adjustROI(cv::Mat &image_gray, TagDetection &tag) {
  float x;
  float y;
  float normdist;
  cv::Point2f p1;
  cv::Point2f p2;

  // calc the roi rect
  p1 = cv::Point2f(tag.p[1].x, tag.p[1].y);
  p2 = cv::Point2f(tag.p[3].x, tag.p[3].y);
  if (this->estimator_initialized) {
    x = this->tag_estimator.mu(0);
    y = this->tag_estimator.mu(1);
  } else {
    x = tag.cxy.x;
    y = tag.cxy.y;
  }

  normdist = cv::norm(p2 - p1);
  this->roi_rect =
    cv::Rect(x - normdist / 2, y - normdist / 2, normdist, normdist);

  this->roi_rect = enlargeROI(image_gray, this->roi_rect, 30);
}

static void mask_image_with_roi(cv::Mat &image,
                                cv::Rect &roi_rect,
                                cv::Mat &masked) {
  cv::Mat mask(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
  cv::rectangle(mask, roi_rect, 255, -1);
  image.copyTo(masked, mask);
}

TagPose Detector::obtainPose(TagDetection &tag, cv::Mat camera_matrix) {
  TagPose tag_pose;

  Eigen::Matrix4d transform;
  Eigen::Matrix4d rotate_transform;
  Eigen::Matrix4d ned_transform;

  cv::Mat R;
  cv::Mat T;
  double tag_size;
  double fx;
  double fy;

  // setup
  tag_size = 0.0;
  fx = camera_matrix.at<double>(0, 0);
  fy = camera_matrix.at<double>(1, 1);

  // get tag size according to tag id
  if (this->tag_sizes.find(tag.id) == this->tag_sizes.end()) {
    // printf("ERROR! Tag size for [%d] not configured!\n", tag.id);
    return tag_pose;
  } else {
    tag_size = this->tag_sizes[tag.id];
  }

  // caculate pose
  CameraUtil::homographyToPoseCV(fx, fy, tag_size, tag.homography, R, T);

  // convert from camera frame (z - forward, x - right, y -down)
  // to NED frame (x - forward, y - right, z -down)
  tag_pose.translation << T.at<double>(2), T.at<double>(0), T.at<double>(1);

  return tag_pose;
}

std::vector<TagPose> Detector::extractTags(cv::Mat &camera_matrix,
                                           cv::Mat &image,
                                           int &timeout) {
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
      pose = this->obtainPose(detections[i], camera_matrix);
      pose_estimates.push_back(pose);
      // this->adjustROI(image, detections[i]);

      // only need 1 tag
      timeout = 0;
      break;
    }
  }

  // no tags
  if (detections.size() == 0) {
    this->roi_rect = cv::Rect(0, 0, image.cols, image.rows);
  }

  return pose_estimates;
}

std::vector<TagPose> Detector::processImage(cv::Mat &camera_matrix,
                                            cv::Mat &image,
                                            int &timeout) {
  TagPose pose;
  cv::Mat image_gray;
  std::vector<TagDetection> tags;
  std::vector<TagPose> pose_estimates;

  // apriltags detector (requires a gray scale image)
  // cv::cvtColor(image, image_gray, CV_BGR2GRAY);

  // create a mask and draw the roi_rect
  // cv::Mat masked(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
  // mask_image_with_roi(image, this->roi_rect, masked);

  // extract tags
  pose_estimates = this->extractTags(camera_matrix, image, timeout);

  // display
  if (this->apriltag_imshow && image.empty() == false) {
    cv::imshow("apriltag detection", image);
    cv::waitKey(1);
  }

  return pose_estimates;
}

std::vector<TagPose> Detector::processImage(cv::Mat &camera_matrix,
                                            cv::Mat &image,
                                            int &timeout,
                                            float dt,
                                            float cam_alpha) {
  std::vector<TagPose> pose_estimates;

  // extract apriltags and estimate pose
  // illuminationInvarientTransform(image, image_gray, cam_alpha);
  pose_estimates = this->extractTags(camera_matrix, image, timeout);

  // display
  if (this->apriltag_imshow && image.empty() == false) {
    cv::imshow("apriltag detection", image);
    cv::waitKey(1);
  }

  return pose_estimates;
}

void Detector::printTag(TagPose &tag) {
  printf("id: %d ", tag.id);
  printf("x=%f ", tag.translation(0));
  printf("y=%f ", tag.translation(1));
  printf("z=%f ", tag.translation(2));
}

}  // end of awesomo namespace
