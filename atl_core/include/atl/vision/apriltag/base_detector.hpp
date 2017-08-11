#ifndef ATL_VISION_APRILTAG_BASE_HPP
#define ATL_VISION_APRILTAG_BASE_HPP

#include <cmath>
#include <fstream>
#include <iostream>
#include <libgen.h>
#include <math.h>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "atl/utils/utils.hpp"
#include "atl/vision/apriltag/data.hpp"
#include "atl/vision/camera/camera.hpp"

namespace atl {

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
  bool windowing;
  double window_padding;
  bool imshow;

  BaseDetector();
  int configure(std::string config_file);
  int illuminationInvariantTransform(cv::Mat &image);
  int changeMode(cv::Mat &image);
  int maskImage(TagPose tag_pose, cv::Mat &image, double padding = 0.5);

  /** Print detected AprilTag */
  void printTag(TagPose tag);
};

} // namespace atl
#endif
