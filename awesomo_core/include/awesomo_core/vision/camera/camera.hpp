#ifndef __AWESOMO_CAMERA_HPP__
#define __AWESOMO_CAMERA_HPP__

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

class CameraConfig {
public:
  int index;
  int image_width;
  int image_height;

  float exposure_value;
  float gain_value;

  cv::Mat camera_matrix;
  cv::Mat rectification_matrix;
  cv::Mat distortion_coefficients;
  cv::Mat projection_matrix;
  Eigen::Matrix<double, 3, 4> projection_matrix_eigen;

  Vec3 lambda;
  float alpha;

  int imshow;
  int snapshot;

  CameraConfig(void);
  int configure(std::string config_file);
};

class Camera {
public:
  CameraConfig *current_config;
  std::map<std::string, CameraConfig *> configs;
  cv::VideoCapture capture;

  Camera(void);
  Camera(int index, int type);
  int configure(std::string config_path);
  int loadMode(std::string mode);

  int getFrame(cv::Mat &image);
  std::vector<TagPose> step(int &timeout);
  std::vector<TagPose> step(int &timeout, float dt);
  int run(void);

  void printConfig(void);
  void printFPS(double &last_tic, int &frame);
};

}  // end of awesomo namespace
#endif
