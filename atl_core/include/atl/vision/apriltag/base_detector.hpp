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
  bool configured = false;

  TagPose prev_tag;
  int prev_tag_image_width = 0;
  int prev_tag_image_height = 0;

  std::map<int, float> tag_configs;
  double tag_sanity_check = FLT_MAX;
  std::string camera_mode;
  std::vector<std::string> camera_modes;
  std::map<std::string, CameraConfig> camera_configs;
  bool illum_invar = false;
  bool windowing = false;
  double window_padding = FLT_MAX;
  bool imshow = false;

  BaseDetector() {}

  /**
   * Configure
   *
   * @param config_file Path to configuration file (YAML)
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Illumination invariant transform
   *
   * @param image Image to be transformed
   * @returns 0 for success, -1 for failure
   */
  int illuminationInvariantTransform(cv::Mat &image);

  /**
   * Change mode
   *
   * @param image Image
   * @returns
   *    - 0: Do not change mode
   *    - 1: Change mode
   */
  int changeMode(const cv::Mat &image);

  /**
   * Mask image
   *
   * @param tag_pose AprilTag pose
   * @param image Image
   * @param padding Mask padding
   *
   * @returns:
   *    - 0: Success
   *    - -1: Current image dimensions != previous image dimensions
   *    - -2: AprilTag ID is not in list of IDs to look out for
   *    - -3: Image dimension / configuration mismatch
   */
  int maskImage(const TagPose &tag_pose,
                const cv::Mat &image,
                const double padding = 0.5);

  /**
   * Print detected AprilTag
   *
   * @param tag AprilTag pose
   */
  void printTag(const TagPose &tag);
};

} // namespace atl
#endif
