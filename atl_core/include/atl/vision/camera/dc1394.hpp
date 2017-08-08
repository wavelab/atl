#ifndef ATL_CORE_VISION_CAMERA_DC1394_HPP
#define ATL_CORE_VISION_CAMERA_DC1394_HPP

#include <stdio.h>

#include <dc1394/dc1394.h>

#include "atl/utils/utils.hpp"
#include "atl/vision/camera/camera.hpp"

namespace atl {

#define CLAMP(x, low, high) \
  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

class DC1394Camera : public Camera {
public:
  dc1394camera_t *capture = nullptr;

  DC1394Camera();
  ~DC1394Camera();

  int initialize();
  void printFrameInfo(dc1394video_frame_t *frame);
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
  // int printFormat7Capabilities();
  // int setFormat7(int mode, std::string pixel_format, int width, int
  // height);
  // std::pair<int, int> centerROI(int size, int max_size, int step);
  // int changeMode(std::string mode);
  int getFrame(cv::Mat &image);
  int run();
};

}  // namespace atl
#endif
