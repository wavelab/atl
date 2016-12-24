#include "awesomo_core/vision/camera/camera.hpp"


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
  YAML::Node config;

  try {
    // pre-check
    if (file_exists(config_file) == false) {
      log_err("File not found: %s", config_file.c_str());
      return -1;
    }

    // setup
    config = YAML::LoadFile(config_file);

    // parse config
    // clang-format off
    if (yamlInteger(config, "index", this->index) != 0) goto error;
    if (yamlInteger(config, "image_width", this->image_width) != 0) goto error;
    if (yamlInteger(config, "image_height", this->image_height) != 0) goto error;

    if (yamlFloat(config, "exposure_value", this->exposure_value) != 0) goto error;
    if (yamlFloat(config, "gain_value", this->gain_value) != 0) goto error;
    if (yamlVec3(config, "lambda", this->lambda) != 0) goto error;
    if (yamlFloat(config, "alpha", this->alpha) != 0) goto error;

    if (yamlCvMat(config, "camera_matrix", this->camera_matrix) != 0) goto error;
    if (yamlCvMat(config, "distortion_coefficients", this->distortion_coefficients) != 0) goto error;
    if (yamlCvMat(config, "rectification_matrix", this->rectification_matrix) != 0) goto error;
    if (yamlCvMat(config, "projection_matrix", this->projection_matrix) != 0) goto error;

    if (yamlBool(config, "imshow", this->imshow) != 0) goto error;
    if (yamlBool(config, "snapshot", this->snapshot) != 0) goto error;
    // clang-format on

    this->loaded = true;

  } catch (YAML::Exception &ex) {
    std::cout << ex.what() << std::endl;
    return -2;
  }

  return 0;
error:
  return -1;
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
  // clang-format on
}

}  // end of awesomo namespace
