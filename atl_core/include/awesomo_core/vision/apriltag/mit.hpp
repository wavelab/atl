#ifndef __atl_VISION_APRILTAG_MIT_HPP__
#define __atl_VISION_APRILTAG_MIT_HPP__

#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <libgen.h>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <apriltags_mit/TagDetector.h>
#include <apriltags_mit/Tag16h5.h>

#include "atl_core/utils/utils.hpp"
#include "atl_core/vision/camera/camera.hpp"
#include "atl_core/vision/apriltag/data.hpp"
#include "atl_core/vision/apriltag/base_detector.hpp"


namespace atl {

class MITDetector : public BaseDetector {
public:
  AprilTags::TagDetector *detector;

  MITDetector(void);
  int configure(std::string config_file);
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);
  int obtainPose(AprilTags::TagDetection tag, TagPose &tag_pose);
};

}  // end of atl namespace
#endif
