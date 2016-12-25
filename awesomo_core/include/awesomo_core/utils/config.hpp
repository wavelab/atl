#ifndef __AWESOMO_UTILS_CONFIG_HPP__
#define __AWESOMO_UTILS_CONFIG_HPP__

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "awesomo_core/utils/io.hpp"
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


enum Type {
  TYPE_NOT_SET = 0,
  // PRIMITIVES
  BOOL = 1,
  INT = 2,
  FLOAT = 3,
  DOUBLE = 4,
  STRING = 5,
  // ARRAY
  BOOL_ARRAY = 11,
  INT_ARRAY = 12,
  FLOAT_ARRAY = 13,
  DOUBLE_ARRAY = 14,
  STRING_ARRAY = 15,
  // VECTOR
  VEC2 = 21,
  VEC3 = 22,
  VEC4 = 23,
  VECX = 24,
  // MATRIX
  MAT2 = 31,
  MAT3 = 32,
  MAT4 = 33,
  MATX = 34,
  CVMAT = 35
};

class ConfigParam {
public:
  enum Type type;
  std::string key;
  bool optional;

  bool *b;
  int *i;
  float *f;
  double *d;
  std::string *s;

  std::vector<bool> *b_array;
  std::vector<int> *i_array;
  std::vector<float> *f_array;
  std::vector<double> *d_array;
  std::vector<std::string> *s_array;

  Vec2 *vec2;
  Vec3 *vec3;
  Vec4 *vec4;
  VecX *vecx;

  Mat2 *mat2;
  Mat3 *mat3;
  Mat4 *mat4;
  MatX *matx;
  cv::Mat *cvmat;

  ConfigParam(void) {
    this->type = TYPE_NOT_SET;
    this->key = "";
    this->optional = false;

    this->b = NULL;
    this->i = NULL;
    this->f = NULL;
    this->d = NULL;
    this->s = NULL;

    this->b_array = NULL;
    this->i_array = NULL;
    this->f_array = NULL;
    this->d_array = NULL;
    this->s_array = NULL;

    this->vec2 = NULL;
    this->vec3 = NULL;
    this->vec4 = NULL;
    this->vecx = NULL;

    this->mat2 = NULL;
    this->mat3 = NULL;
    this->mat4 = NULL;
    this->matx = NULL;
    this->cvmat = NULL;
  }
};

class ConfigParser {
public:
  bool configured;
  bool loaded;

  YAML::Node root;
  std::vector<ConfigParam> params;

  ConfigParser(void);
  void addParam(enum Type type, std::string key, void *out);
  int checkKey(std::string key, bool optional);
  int loadPrimitive(ConfigParam param);
  int loadArray(ConfigParam param);
  int loadVector(ConfigParam param);
  int loadMatrix(ConfigParam param);
  int load(std::string config_file);
};

}  // end of awesomo namespace
#endif
