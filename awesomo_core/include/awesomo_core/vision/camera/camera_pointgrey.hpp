#ifndef __AWESOMO_CORE_VISION_CAMERA_POINTGREY_HPP__
#define __AWESOMO_CORE_VISION_CAMERA_POINTGREY_HPP__

#include <FlyCapture2.h>


namespace awesomo {

class PointGreyCamera : public Camera {
public:
  bool configured;
  FlyCapture2::Camera *firefly;

  PointGreyCamera(void);
  int configure(void);
  int initialize(void);
};

}  // end of awesomo namespace
#endif
