#include "atl/vision/camera/config.hpp"

namespace atl {

int CameraConfig::load(const std::string &config_file) {
  ConfigParser parser;

  // config variables
  // clang-format off
  parser.addParam("index", &this->index);
  parser.addParam("image_width", &this->image_width);
  parser.addParam("image_height", &this->image_height);

  parser.addParam("shutter_speed", &this->shutter_speed, true);
  parser.addParam("exposure_value", &this->exposure_value, true);
  parser.addParam("gain_value", &this->gain_value, true);

  parser.addParam("lambda", &this->lambda);
  parser.addParam("alpha", &this->alpha);

  parser.addParam("camera_matrix", &this->camera_matrix);
  parser.addParam("distortion_coefficients", &this->distortion_coefficients);
  parser.addParam("rectification_matrix", &this->rectification_matrix);
  parser.addParam("projection_matrix", &this->projection_matrix);

  parser.addParam("imshow", &this->imshow);
  parser.addParam("snapshot", &this->snapshot);
  parser.addParam("showfps", &this->showfps);
  // clang-format on

  // load config
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load configure file [%s]!", config_file.c_str());
    return -1;
  }

  this->loaded = true;
  return 0;
}

void CameraConfig::print() {
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

} // namespace atl
