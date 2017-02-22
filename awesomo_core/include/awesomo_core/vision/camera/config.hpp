#ifndef __AWESOMO_VISION_CAMERA_CONFIG_HPP__
#define __AWESOMO_VISION_CAMERA_CONFIG_HPP__

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

class CameraConfig {
public:
  bool loaded;

  int index;
  int image_width;
  int image_height;

  float shutter_speed;
  float exposure_value;
  float gain_value;
  Vec3 lambda;
  float alpha;

  cv::Mat camera_matrix;
  cv::Mat rectification_matrix;
  cv::Mat distortion_coefficients;
  cv::Mat projection_matrix;

  bool imshow;
  bool snapshot;
  bool showfps;

  CameraConfig(void);
  int load(std::string config_file);
  void print(void);
};

}  // end of awesomo namespace
#endif
