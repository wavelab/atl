#ifndef __AWESOMO_VISION_CAMERA_HPP__
#define __AWESOMO_VISION_CAMERA_HPP__

#include <algorithm>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/vision/camera/config.hpp"


namespace awesomo {

class Camera {
public:
  bool configured;
  bool initialized;

  CameraConfig config;
  std::vector<std::string> modes;
  std::map<std::string, CameraConfig> configs;

  cv::Mat image;
  double last_tic;

  cv::VideoCapture *capture;

  Camera(void);
  ~Camera(void);
  virtual int configure(std::string config_path);
  virtual int initialize(void);
  virtual int shutdown(void);
  virtual int changeMode(std::string mode);
  virtual int getFrame(cv::Mat &image);
  int run(void);
  int showFPS(double &last_tic, int &frame);
  int showImage(cv::Mat &image);
};

}  // end of awesomo namespace
#endif
