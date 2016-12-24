#ifndef __AWESOMO_UTILS_CONFIG_HPP__
#define __AWESOMO_UTILS_CONFIG_HPP__

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "awesomo_core/utils/math.hpp"
#include "awesomo_core/utils/logging.hpp"


namespace awesomo {

#define YAML_PRIMITIVE_RETVAL(X)                               \
  if (retval == -1) {                                          \
    if (optional == false) {                                   \
      log_err("Opps [%s] missing in yaml file!", key.c_str()); \
      return -1;                                               \
    } else {                                                   \
      return 0;                                                \
    }                                                          \
  }

#define YAML_VECTOR_RETVAL(X)                                  \
  if (retval == -1) {                                          \
    if (optional == false) {                                   \
      log_err("Opps [%s] missing in yaml file!", key.c_str()); \
      return -1;                                               \
    } else {                                                   \
      return 0;                                                \
    }                                                          \
  } else if (retval == -2) {                                   \
    log_err("Invalid [%s] vector!", key.c_str());              \
    return -2;                                                 \
  }

#define YAML_MATRIX_RETVAL(X)                                  \
  if (retval == -1) {                                          \
    if (optional == false) {                                   \
      log_err("Opps [%s] missing in yaml file!", key.c_str()); \
      return -1;                                               \
    } else {                                                   \
      return 0;                                                \
    }                                                          \
  } else if (retval == -2) {                                   \
    log_err("Invalid [%s] matrix!", key.c_str());              \
    return -2;                                                 \
  }

// clang-format off
int yamlCheckKey(YAML::Node yaml, std::string key);
int yamlCheckVector(YAML::Node yaml, std::string key);
int yamlCheckMatrix(YAML::Node yaml, std::string key);

int yamlBool(YAML::Node yaml, std::string key, bool &x, bool optional = false);
int yamlInteger(YAML::Node yaml, std::string key, int &x, bool optional = false);
int yamlFloat(YAML::Node yaml, std::string key, float &x, bool optional = false);
int yamlDouble(YAML::Node yaml, std::string key, double &x, bool optional = false);
int yamlString(YAML::Node yaml, std::string key, std::string &x, bool optional = false);

int yamlVec2(YAML::Node yaml, std::string key, Vec3 &x, bool optional = false);
int yamlVec3(YAML::Node yaml, std::string key, Vec3 &x, bool optional = false);
int yamlVec4(YAML::Node yaml, std::string key, Vec4 &x, bool optional = false);
int yamlVecX(YAML::Node yaml, std::string key, VecX &x, bool optional = false);

int yamlMat2(YAML::Node yaml, std::string key, Mat2 &x, bool optional = false);
int yamlMat3(YAML::Node yaml, std::string key, Mat3 &x, bool optional = false);
int yamlMatX(YAML::Node yaml, std::string key, MatX &x, bool optional = false);

int yamlCvMat(YAML::Node yaml, std::string key, cv::Mat &x, bool optional = false);
// clang-format on

}  // end of awesomo namespace
#endif
