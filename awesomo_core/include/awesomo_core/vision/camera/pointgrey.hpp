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
  int setBrightness(double brightness);
  int setFrameRate(double fps);
  int setExposure(double exposure);
  int setShutter(double shutter_ms);
  int setGain(double gain_db);
  int getBrightness(double &brightness);
  int getFrameRate(double &fps);
  int getExposure(double &exposure);
  int getShutter(double &shutter_ms);
  int getGain(double &gain_db);
  int printFormat7Capabilities(void);
  int setFormat7(int mode, std::string pixel_format, int width, int height);
  std::pair<int, int> centerROI(int size, int max_size, int step);
  int changeMode(std::string mode);
  int getFrame(cv::Mat &image);
  int run(void);
};

}  // end of awesomo namespace
#endif
