#ifndef __AWESOMO_CORE_VISION_CAMERA_XIMEA_HPP__
#define __AWESOMO_CORE_VISION_CAMERA_XIMEA_HPP__

#include <m3api/xiApi.h>

#include "awesomo_core/vision/camera/camera.hpp"


namespace awesomo {

#define XIMEA_CHECK(STATE, WHERE)                \
  if (STATE != XI_OK) {                          \
    printf("Error after %s (%d)", WHERE, STATE); \
    goto ximea_error;                            \
  }

class XimeaCamera : public Camera {
public:
  HANDLE ximea;

  XimeaCamera(void) {};
  int initialize(void);
  int getFrame(cv::Mat &image);
};

}  // end of awesomo namespace
#endif
