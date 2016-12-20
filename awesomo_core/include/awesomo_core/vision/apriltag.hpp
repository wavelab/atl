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

#include "awesomo_core/vision/util.hpp"
#include "awesomo_core/vision/estimator.hpp"


#define _USE_MATH_DEFINES
#ifdef M_PI
#define TWOPI 2 * M_PI
#else
#define TWOPI 2.0 * 3.1415926535897932384626433832795
#endif


// CLASSES
class TagPose {
public:
  int id;
  Eigen::Vector3d translation;
};

class Detector {
public:
  // properties
  TagDetector *detector;
  std::map<int, float> tag_sizes;

  int apriltag_imshow;
  cv::Rect roi_rect;

  struct kf tag_estimator;
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
