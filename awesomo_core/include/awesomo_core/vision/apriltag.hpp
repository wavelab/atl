#ifndef __AWESOMO_APRILTAG_HPP__
#define __AWESOMO_APRILTAG_HPP__

#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <apriltags/TagDetector.h>
#include <apriltags/CameraUtil.h>

#include "awesomo_core/utils/utils.hpp"
// #include "awesomo_core/vision/estimator.hpp"


#define _USE_MATH_DEFINES
#ifdef M_PI
#define TWOPI 2 * M_PI
#else
#define TWOPI 2.0 * 3.1415926535897932384626433832795
#endif

class TagPose {
public:
  int id;
  bool detected;
  Eigen::Vector3d position;

  TagPose(void) {
    this->id = -1;
    this->detected = false;
    this->position << 0.0, 0.0, 0.0;
  };
  TagPose(int id, bool detected, Eigen::Vector3d position) {
    this->id = id;
    this->detected = detected;
    this->position = position;
  }
};

class Detector {
public:
  // properties
  TagDetector *detector;
  std::map<int, float> tag_sizes;

  int apriltag_imshow;
  cv::Rect roi_rect;

  // struct kf tag_estimator;
  bool use_estimator;
  bool estimator_initialized;

  Eigen::Matrix<double, 3, 4> projection_matrix;
  Eigen::Vector3d image_target_center;
  Eigen::Vector3d image_target_top_left;
  Eigen::Vector3d image_target_bottom_right;

  // constructor
  Detector(void);
  Detector(int apriltag_imshow);

  // methods
  void adjustROI(cv::Mat &image_gray, TagDetection &tag);
  TagPose obtainPose(TagDetection &detection, cv::Mat camera_matrix);
  std::vector<TagPose> extractTags(cv::Mat &camera_matrix,
                                   cv::Mat &image,
                                   int &timeout);
  std::vector<TagPose> processImage(cv::Mat &camera_matrix,
                                    cv::Mat &image,
                                    int &timeout);
  std::vector<TagPose> processImage(cv::Mat &camera_matrix,
                                    cv::Mat &image,
                                    int &timeout,
                                    float dt,
                                    float cam_alpha);
  void printTag(TagPose &tag);
};

#endif
