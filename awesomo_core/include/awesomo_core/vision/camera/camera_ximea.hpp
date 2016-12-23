#ifndef __AWESOMO_CORE_VISION_CAMERA_XIMEA_HPP__
#define __AWESOMO_CORE_VISION_CAMERA_XIMEA_HPP__

#include <m3api/xiApi.h>


namespace awesomo {

class XimeaCamera : public Camera {
public:
  bool configured;
  HANDLE ximea;

  XimeaCamera(void);
  int configure(void);
  int initialize(void);
};

}  // end of awesomo namespace
#endif
