#ifndef __AWESOMO_CORE_VISION_CAMERA_POINTGREY_HPP__
#define __AWESOMO_CORE_VISION_CAMERA_POINTGREY_HPP__

#include <flycapture/FlyCapture2.h>
#include "awesomo_core/vision/camera/camera.hpp"


namespace awesomo {

class PointGreyCamera : public Camera {
public:
  FlyCapture2::Camera *pointgrey;
  double shutter_speed;

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
  int setProperty(const FlyCapture2::PropertyType prop_type,
                  bool on,
                  bool auto_on,
                  double value);
  FlyCapture2::Property getProperty(const FlyCapture2::PropertyType &prop_type);
  FlyCapture2::PropertyInfo getPropertyInfo(const FlyCapture2::PropertyType &prop_type);
  int setFormat7(std::string pixel_format,
                 int crop_width,
                 int crop_height,
                 int mode);
  int setFormat7VideoMode(int format7_mode,
                          std::string pixel_format,
                          int width,
                          int height,
                          bool center_ROI);
  int setROI(const FlyCapture2::Format7Info &format7_info,
             FlyCapture2::Format7ImageSettings &format7_settings,
             int width,
             int height,
             bool center_ROI);
  std::pair<int, int> centerROI(int size, int max_size, int step);
  int changeMode(std::string mode);
  int getFrame(cv::Mat &image);
  int run(void);
};

}  // end of awesomo namespace
#endif
