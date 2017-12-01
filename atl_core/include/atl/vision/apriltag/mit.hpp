#ifndef ATL_VISION_APRILTAG_MIT_HPP
#define ATL_VISION_APRILTAG_MIT_HPP

#include <cmath>
#include <fstream>
#include <iostream>
#include <libgen.h>
#include <math.h>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// clang-format off
#include <apriltags_mit/TagDetector.h>
#include <apriltags_mit/Tag16h5.h>
// clang-format on

#include "atl/utils/utils.hpp"
#include "atl/vision/apriltag/base_detector.hpp"
#include "atl/vision/apriltag/data.hpp"
#include "atl/vision/camera/camera.hpp"

namespace atl {

/**
 * MIT Apriltag Detector
 **/
class MITDetector : public BaseDetector {
public:
  AprilTags::TagDetector *detector = nullptr;

  MITDetector() {}

  /**
   * Configure
   *
   * @param config_file Path to configuration file (YAML)
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Extract AprilTags
   *
   * @param image
   * @param tags
   * @returns 0 for success else failure
   */
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);
};

} // namespace atl
#endif
