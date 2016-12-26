#ifndef __AWESOMO_CORE_VISION_CAMERA_POINTGREY_HPP__
#define __AWESOMO_CORE_VISION_CAMERA_POINTGREY_HPP__

#include <FlyCapture2.h>

#include "awesomo_core/vision/camera/camera_pointgrey.hpp"


namespace awesomo {

class PointGreyCamera : public Camera {
public:
  FlyCapture2::Camera *pointgrey;

  PointGreyCamera(void);
  ~PointGreyCamera(void);
  int initialize(void);
  int getFrame(cv::Mat &image);
};

}  // end of awesomo namespace
#endif
