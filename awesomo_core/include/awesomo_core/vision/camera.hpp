#ifndef __AWESOMO_CORE_VISION_CAMERA_HPP__
#define __AWESOMO_CORE_VISION_CAMERA_HPP__

#include <fstream>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <memory.h>
#include <sys/time.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <FlyCapture2.h>

#include "awesomo_core/utils/utils.hpp"
// #include "awesomo_core/vision/gimbal.hpp"
// #include "awesomo_core/vision/apriltag.hpp"
// #include "awesomo_core/vision/estimator.hpp"


// CONSTANTS
#define CAMERA_NORMAL 0
#define CAMERA_FIREFLY 1
#define CAMERA_XIMEA 2


namespace awesomo {

// CLASSES
class CameraConfig {
public:
  int camera_mode;
  int image_width;
  int image_height;

  cv::Mat camera_matrix;
  cv::Mat rectification_matrix;
  cv::Mat distortion_coefficients;
  cv::Mat projection_matrix;
  Eigen::Matrix<double, 3, 4> projection_matrix_eigen;
};

class Camera {
public:
  Detector *tag_detector;

  int index;
  int type;
  int imshow;
  int snapshot;
  float exposure_value;
  float gain_value;
  std::string mode;
  Vec3 lambda;
  float alpha;

  CameraConfig *config;
  std::map<std::string, CameraConfig *> configs;
  std::vector<std::string> config_keys;
  std::vector<std::string> config_values;
  std::vector<float> config_dists;

  cv::VideoCapture *capture;
  FlyCapture2::Camera *capture_firefly;
  HANDLE ximea;

  Camera(int index, int type);
  Camera(std::string config_path);

  CameraConfig *loadConfig(std::string mode, std::string calib_file);
  int loadConfig(std::string mode);
  int setLambdas(float lambda_1, float lambda_2, float lambda_3);
  void adjustMode(std::vector<TagPose> &pose_estimates, int &timeout);

  void printConfig(void);
  void printFPS(double &last_tic, int &frame);

  int getFrame(cv::Mat &image);

  int run(void);
  void trackTarget(std::vector<TagPose> pose_estimates);
  std::vector<TagPose> step(int &timeout);
  std::vector<TagPose> step(int &timeout, float dt);
};

}  // end of awesomo namespace
#endif
