#include "awesomo_core/vision/camera/config.hpp"


namespace awesomo {

CameraConfig::CameraConfig(void) {
  this->loaded = false;

  this->index = 0;
  this->image_width = 0;
  this->image_height = 0;

  this->shutter_speed = 0.0;
  this->exposure_value = 0.0;
  this->gain_value = 0.0;
  this->lambda << 0.0, 0.0, 0.0;
  this->alpha = 0.0;

  this->camera_matrix;
  this->rectification_matrix;
  this->distortion_coefficients;
  this->projection_matrix;

  this->imshow = false;
  this->snapshot = false;
}

int CameraConfig::load(std::string config_file) {
  ConfigParser parser;

  // config variables
  // clang-format off
  parser.addParam<int>("index", &this->index);
  parser.addParam<int>("image_width", &this->image_width);
  parser.addParam<int>("image_height", &this->image_height);

  parser.addParam<float>("shutter_speed", &this->shutter_speed, true);
  parser.addParam<float>("exposure_value", &this->exposure_value);
  parser.addParam<float>("gain_value", &this->gain_value);
  parser.addParam<Vec3>("lambda", &this->lambda);
  parser.addParam<float>("alpha", &this->alpha);

  parser.addParam<cv::Mat>("camera_matrix", &this->camera_matrix);
  parser.addParam<cv::Mat>("distortion_coefficients", &this->distortion_coefficients);
  parser.addParam<cv::Mat>("rectification_matrix", &this->rectification_matrix);
  parser.addParam<cv::Mat>("projection_matrix", &this->projection_matrix);

  parser.addParam<bool>("imshow", &this->imshow);
  parser.addParam<bool>("snapshot", &this->snapshot);
  parser.addParam<bool>("showfps", &this->showfps);
  // clang-format on

  // load config
  if (parser.load(config_file) != 0) {
    log_err("Failed to load configure file [%s]!", config_file.c_str());
    return -1;
  }

  this->loaded = true;
  return 0;
}

void CameraConfig::print(void) {
  // clang-format off
  std::cout << "index: " << this->index << std::endl;
  std::cout << "image_width: " << this->image_width << std::endl;
  std::cout << "image_height: " << this->image_height << std::endl;

  std::cout << "exposure_value: " << this->exposure_value << std::endl;
  std::cout << "gain_value: " << this->gain_value << std::endl;
  std::cout << "lambda: " << this->lambda.transpose() << std::endl;
  std::cout << "alpha: " << this->alpha << std::endl;

  std::cout << "camera_matrix: \n" << this->camera_matrix << std::endl;
  std::cout << "rectification_matrix: \n" << this->rectification_matrix << std::endl;
  std::cout << "distortion_coefficients: " << this->distortion_coefficients << std::endl;
  std::cout << "projection_matrix: \n" << this->projection_matrix << std::endl;

  std::cout << "imshow: " << this->imshow << std::endl;
  std::cout << "snapshot: " << this->snapshot << std::endl;
  std::cout << "showfps: " << this->showfps << std::endl;
  // clang-format on
}

}  // end of awesomo namespace
