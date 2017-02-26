#ifndef __AWESOMO_VISION_APRILTAG_MIT_HPP__
#define __AWESOMO_VISION_APRILTAG_MIT_HPP__

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

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/vision/camera/camera.hpp"
#include "awesomo_core/vision/apriltag/data.hpp"
#include "awesomo_core/vision/apriltag/base_detector.hpp"


namespace awesomo {

class MITDetector : public BaseDetector {
public:
  AprilTags::TagDetector *detector;

  MITDetector(void);
  int configure(std::string config_file);
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);
  int obtainPose(AprilTags::TagDetection tag, TagPose &tag_pose);
};

}  // end of awesomo namespace
#endif
