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
  int getFrame(cv::Mat &image);
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
  int setFrameRate(bool auto_frame_rate, double frame_rate);
  int setExposure(bool auto_exposure, double exposure);
  int setGain(bool auto_gain, double gain);
  int setShutter(bool auto_shutter, double shutter);
  int printFormat7Capabilities(void);
  int changeMode(std::string mode);
  int setProperty(const FlyCapture2::PropertyType prop_type,
                  bool on,
                  bool auto_on,
                  double value);
  FlyCapture2::Property getProperty(const FlyCapture2::PropertyType &prop_type);
  FlyCapture2::PropertyInfo getPropertyInfo(const FlyCapture2::PropertyType &prop_type);
};

}  // end of awesomo namespace
#endif
