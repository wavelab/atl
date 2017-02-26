#ifndef __AWESOMO_VISION_APRILTAG_SWATHMORE_HPP__
#define __AWESOMO_VISION_APRILTAG_SWATHMORE_HPP__

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

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/vision/camera/camera.hpp"
#include "awesomo_core/vision/apriltag/data.hpp"
#include "awesomo_core/vision/apriltag/base_detector.hpp"


namespace awesomo {

class SwathmoreDetector : public BaseDetector {
public:
  TagDetector *detector;

  SwathmoreDetector(void);
  int configure(std::string config_file);
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);
  int obtainPose(TagDetection tag, TagPose &tag_pose);
};

}  // end of awesomo namespace
#endif
