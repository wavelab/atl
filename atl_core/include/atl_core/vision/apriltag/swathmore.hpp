#ifndef ATL_VISION_APRILTAG_SWATHMORE_HPP
#define ATL_VISION_APRILTAG_SWATHMORE_HPP

#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <libgen.h>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <apriltags_swathmore/TagDetector.h>
#include <apriltags_swathmore/CameraUtil.h>

#include "atl_core/utils/utils.hpp"
#include "atl_core/vision/camera/camera.hpp"
#include "atl_core/vision/apriltag/data.hpp"
#include "atl_core/vision/apriltag/base_detector.hpp"


namespace atl {

class SwathmoreDetector : public BaseDetector {
public:
  TagDetector *detector;

  SwathmoreDetector(void);
  int configure(std::string config_file);
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);
  int obtainPose(TagDetection tag, TagPose &tag_pose);
};

}  // end of atl namespace
#endif
