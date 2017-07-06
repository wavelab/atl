#ifndef ATL_VISION_CAMERA_HPP
#define ATL_VISION_CAMERA_HPP

#include <algorithm>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "atl/utils/utils.hpp"
#include "atl/vision/camera/config.hpp"

namespace atl {

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

}  // namespace atl
#endif
