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
  int checkVector(std::string key, bool optional);
  int checkMatrix(std::string key, bool optional);
  int loadPrimitive(ConfigParam param);
  int loadArray(ConfigParam param);
  int loadVector(ConfigParam param);
  int loadMatrix(ConfigParam param);
  int load(std::string config_file);
};

}  // end of awesomo namespace
#endif
