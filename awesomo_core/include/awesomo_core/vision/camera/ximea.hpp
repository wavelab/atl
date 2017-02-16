#ifndef __AWESOMO_CORE_VISION_CAMERA_XIMEA_HPP__
#define __AWESOMO_CORE_VISION_CAMERA_XIMEA_HPP__

#include <m3api/xiApi.h>

#include "awesomo_core/vision/camera/camera.hpp"


namespace awesomo {

// #define XIMEA_CHECK(STATE, WHERE)                \
//   if (STATE != XI_OK) {                          \
//     printf("Error after %s (%d)", WHERE, STATE); \
//     goto ximea_error;                            \
//   }

#define XIMEA_CHECK(STATE, WHERE)                \
  if (STATE != XI_OK) {                          \
    printf("Error after %s (%d)", WHERE, STATE); \
    ximea_error();                            \
  }

class XimeaCamera : public Camera {
public:
  HANDLE ximea;

  XimeaCamera(void);

  int initialize(void);
  int setExposure(float exposure_time_us);
  int setGain(float gain_db);
  int getFrame(cv::Mat &image);
  int changeMode(std::string mode);
  int checkState(XI_RETURN retval, std::string where);
};

}  // end of awesomo namespace
#endif
