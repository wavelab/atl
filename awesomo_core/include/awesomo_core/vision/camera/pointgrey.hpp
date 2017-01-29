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
  int setFormat7(std::string pixel_format,
                 int crop_width,
                 int crop_height,
                 int mode);
  int setFrameRate(double frame_rate);
  int setExposure(double exposure);
  int setGain(double gain);
  int printFormat7Capabilities(void);
};

}  // end of awesomo namespace
#endif
