#ifndef ATL_VISION_APRILTAG_MICHIGAN_HPP
#define ATL_VISION_APRILTAG_MICHIGAN_HPP

#include <cmath>
#include <fstream>
#include <iostream>
#include <libgen.h>
#include <math.h>
#include <sys/time.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>

#include "atl/utils/utils.hpp"
#include "atl/vision/apriltag/base_detector.hpp"
#include "atl/vision/apriltag/data.hpp"
#include "atl/vision/camera/camera.hpp"

namespace atl {

/** Michigan Apriltag Detector **/
class MichiganDetector : public BaseDetector {
public:
  apriltag_detector_t *detector;
  apriltag_family_t *family;

  MichiganDetector();
  ~MichiganDetector();

  /**
   * Configure
   * @param config_file Path to configuration file (YAML)
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Extract AprilTags
   * @param image
   * @param tags
   * @returns 0 for success else failure
   */
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);

  /**
   * Obtain pose
   * @param tag Tag detected
   * @param tag_pose Tag Pose
   * @returns 0 for success and -1 for failure
   */
  int obtainPose(apriltag_detection_t *tag, TagPose &tag_pose);
};

} // namespace atl
#endif
