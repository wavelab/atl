#ifndef __AWESOMO_VISION_APRILTAG_MICHIGAN_HPP__
#define __AWESOMO_VISION_APRILTAG_MICHIGAN_HPP__

#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <libgen.h>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/vision/camera/camera.hpp"
#include "awesomo_core/vision/apriltag/data.hpp"
#include "awesomo_core/vision/apriltag/base_detector.hpp"


namespace awesomo {

class MichiganDetector : public BaseDetector {
public:
  apriltag_detector_t *detector;
  apriltag_family_t *family;

  MichiganDetector(void);
  ~MichiganDetector(void);
  int configure(std::string config_file);
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);
  int obtainPose(apriltag_detection_t *det, TagPose &tag_pose);
};

}  // end of awesomo namespace
#endif
