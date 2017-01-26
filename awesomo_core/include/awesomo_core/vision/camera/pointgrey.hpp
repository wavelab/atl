#ifndef __AWESOMO_CORE_VISION_CAMERA_POINTGREY_HPP__
#define __AWESOMO_CORE_VISION_CAMERA_POINTGREY_HPP__

#include <flycapture/FlyCapture2.h>
#include "awesomo_core/vision/camera/camera.hpp"


namespace awesomo {

class PointGreyCamera : public Camera {
public:
  FlyCapture2::Camera *pointgrey;

  PointGreyCamera(void);
  ~PointGreyCamera(void);
  int initialize(void);
  int getFrame(cv::Mat &image);
  int printFormat7Capabilities(void);
  int setFormat7(void);
  int setFrameRate(double fps);
};

}  // end of awesomo namespace
#endif
