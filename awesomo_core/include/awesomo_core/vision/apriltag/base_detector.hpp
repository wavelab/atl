#ifndef __AWESOMO_VISION_APRILTAG_BASE_HPP__
#define __AWESOMO_VISION_APRILTAG_BASE_HPP__

#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <libgen.h>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/vision/camera/camera.hpp"
#include "awesomo_core/vision/apriltag/data.hpp"


namespace awesomo {

class BaseDetector {
public:
  bool configured;

  TagPose prev_tag;
  int prev_tag_image_width;
  int prev_tag_image_height;

  std::map<int, float> tag_configs;
  double tag_sanity_check;
  std::string camera_mode;
  std::vector<std::string> camera_modes;
  std::map<std::string, CameraConfig> camera_configs;
  bool illum_invar;
  bool imshow;

  BaseDetector(void);
  int configure(std::string config_file);
  int illuminationInvariantTransform(cv::Mat &image);
  int changeMode(cv::Mat &image);
  int maskImage(TagPose tag_pose, cv::Mat &image, double padding=0.5);
  void printTag(TagPose tag);
};

}  // end of awesomo namespace
#endif
