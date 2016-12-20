#ifndef __AWESOMO_CAMERA_HPP__
#define __AWESOMO_CAMERA_HPP__

#include <fstream>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <memory.h>
#include <sys/time.h>

#include <ros/console.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <FlyCapture2.h>
#include <m3api/xiApi.h>

#include "awesomo_core/vision/util.hpp"
#include "awesomo_core/vision/gimbal.hpp"
#include "awesomo_core/vision/apriltag.hpp"
#include "awesomo_core/vision/estimator.hpp"


// CONSTANTS
#define CAMERA_NORMAL 0
#define CAMERA_FIREFLY 1
#define CAMERA_XIMEA 2


// MACROS
#define XIMEA_CHECK(STATE, WHERE)                \
  if (STATE != XI_OK) {                          \
    printf("Error after %s (%d)", WHERE, STATE); \
    goto ximea_error;                            \
  }


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

  int camera_index;
  int camera_type;
  int camera_imshow;
  int camera_snapshot;
  float camera_exposure_value;
  float camera_gain_value;
  std::string camera_mode;
  float lambda_1;
  float lambda_2;
  float lambda_3;
  float alpha;

  CameraConfig *config;
  std::map<std::string, CameraConfig *> configs;
  std::vector<std::string> config_keys;
  std::vector<std::string> config_values;
  std::vector<float> config_dists;

  cv::VideoCapture *capture;
  FlyCapture2::Camera *capture_firefly;
  HANDLE ximea;

  Gimbal *gimbal;
  struct kf tag_estimator;
  bool tag_estimator_initialized;
  struct timespec tag_last_seen;
  struct timespec tag_estimator_last_updated;

  Camera(int camera_index, int camera_type);
  Camera(std::string camera_config_path);

  int initWebcam(int image_width, int image_height);
  int initFirefly(void);
  int initXimea(void);
  int initCamera(std::string camera_mode);
  int initGimbal(std::string config_path);

  CameraConfig *loadConfig(std::string mode, const std::string calib_file);
  int loadConfig(std::string camera_mode);
  int setLambdas(float lambda_1, float lambda_2, float lambda_3);
  void adjustMode(std::vector<TagPose> &pose_estimates, int &timeout);

  void printConfig(void);
  void printFPS(double &last_tic, int &frame);

  int getFrameWebcam(cv::Mat &image);
  int getFramePointGrey(cv::Mat &image);
  int getFrameXimea(cv::Mat &image);
  int getFrame(cv::Mat &image);

  int run(void);
  void trackTarget(std::vector<TagPose> pose_estimates);
  std::vector<TagPose> step(int &timeout);
  std::vector<TagPose> step(int &timeout, float dt);
};

#endif
