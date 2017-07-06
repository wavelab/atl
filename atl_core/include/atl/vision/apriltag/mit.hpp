#ifndef ATL_VISION_APRILTAG_MIT_HPP
#define ATL_VISION_APRILTAG_MIT_HPP

#include <libgen.h>
#include <math.h>
#include <sys/time.h>
#include <cmath>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <apriltags_mit/Tag16h5.h>
#include <apriltags_mit/TagDetector.h>

#include "atl/utils/utils.hpp"
#include "atl/vision/apriltag/base_detector.hpp"
#include "atl/vision/apriltag/data.hpp"
#include "atl/vision/camera/camera.hpp"

namespace atl {

class MITDetector : public BaseDetector {
public:
  AprilTags::TagDetector *detector;

  MITDetector(void);
  int configure(std::string config_file);
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);
  int obtainPose(AprilTags::TagDetection tag, TagPose &tag_pose);
};

}  // namespace atl
#endif
