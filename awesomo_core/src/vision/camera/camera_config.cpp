#include "awesomo_core/vision/camera/camera_config.hpp"


namespace awesomo {

CameraConfig::CameraConfig(void) {
  this->loaded = false;

  this->index = 0;
  this->image_width = 0;
  this->image_height = 0;

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
  parser.addParam(INT, "index", &this->index);
  parser.addParam(INT, "image_width", &this->image_width);
  parser.addParam(INT, "image_height", &this->image_height);

  parser.addParam(FLOAT, "exposure_value", &this->exposure_value);
  parser.addParam(FLOAT, "gain_value", &this->gain_value);
  parser.addParam(VEC3, "lambda", &this->lambda);
  parser.addParam(FLOAT, "alpha", &this->alpha);

  parser.addParam(CVMAT, "camera_matrix", &this->camera_matrix);
  parser.addParam(CVMAT, "distortion_coefficients", &this->distortion_coefficients);
  parser.addParam(CVMAT, "rectification_matrix", &this->rectification_matrix);
  parser.addParam(CVMAT, "projection_matrix", &this->projection_matrix);

  parser.addParam(BOOL, "imshow", &this->imshow);
  parser.addParam(BOOL, "snapshot", &this->snapshot);
  parser.addParam(BOOL, "showfps", &this->showfps);
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
