#ifndef ATL_VISION_APRILTAG_MICHIGAN_HPP
#define ATL_VISION_APRILTAG_MICHIGAN_HPP

#include <libgen.h>
#include <math.h>
#include <sys/time.h>
#include <cmath>
#include <fstream>
#include <iostream>

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

class MichiganDetector : public BaseDetector {
public:
  apriltag_detector_t *detector;
  apriltag_family_t *family;

  MichiganDetector();
  ~MichiganDetector();
  int configure(std::string config_file);
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);
  int obtainPose(apriltag_detection_t *det, TagPose &tag_pose);
};

}  // namespace atl
#endif
