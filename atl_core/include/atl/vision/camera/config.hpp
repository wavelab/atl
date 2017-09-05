#ifndef ATL_VISION_CAMERA_CONFIG_HPP
#define ATL_VISION_CAMERA_CONFIG_HPP

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "atl/utils/utils.hpp"

namespace atl {

class CameraConfig {
public:
  bool loaded = false;

  int index = 0;
  int image_width = 0;
  int image_height = 0;
  std::string image_type = "bgr8";

  bool roi = false;
  int roi_width = 0;
  int roi_height = 0;

  float shutter_speed = 0.0;
  float exposure_value = 0.0;
  float gain_value = 0.0;

  Vec3 lambda{0.0, 0.0, 0.0};
  float alpha = 0.0;

  cv::Mat camera_matrix;
  cv::Mat rectification_matrix;
  cv::Mat distortion_coefficients;
  cv::Mat projection_matrix;

  bool imshow = false;
  bool snapshot = false;
  bool showfps = false;

  CameraConfig() {}

  /**
   * Load camera config
   *
   * @param config_file Path to config file
   * @return 0 for success, -1 for failure
   */
  int load(const std::string &config_file);

  /**
   * Print camera config
   */
  void print();
};

} // namespace atl
#endif
