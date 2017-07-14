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

/** Generic Camera **/
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

  Camera();
  ~Camera();

  /**
   * Configure camera
   * @param config_path Path to config file (YAML)
   * @returns 0 for success, -1 for failure
   */
  virtual int configure(const std::string &config_path);

  /**
   * Initialize camera
   * @returns 0 for success, -1 for failure
   */
  virtual int initialize();

  /**
   * Shutdown camera
   * @returns 0 for success, -1 for failure
   */
  virtual int shutdown();

  /**
   * Change camera mode
   * @returns 0 for success, -1 for failure
   */
  virtual int changeMode(const std::string &mode);

  /**
   * Get camera frame
   * @params image Camera frame image
   * @returns 0 for success, -1 for failure
   */
  virtual int getFrame(cv::Mat &image);

  /**
   * Run camera
   * @returns 0 for success, -1 for failure
   */
  int run();

  /**
   * Show FPS
   * @params last_tic Last tic in seconds
   * @params frame Frame number
   * @returns 0 for success, -1 for failure
   */
  int showFPS(double &last_tic, int &frame);

  /**
   * Show image
   * @params image Image
   * @returns 0 for success, -1 for failure
   */
  int showImage(cv::Mat &image);
};

}  // namespace atl
#endif
