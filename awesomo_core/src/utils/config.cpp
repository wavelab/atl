#include "awesomo_core/utils/config.hpp"


namespace awesomo {

int yamlCheckKey(YAML::Node yaml, std::string key) {
  if (!yaml[key]) {
    return -1;
  }

  return 0;
}

int yamlCheckVector(YAML::Node yaml, std::string key) {
  int retval;
  const std::string targets[3] = {"rows", "cols", "data"};

  // check key
  retval = yamlCheckKey(yaml, key);
  if (retval == -1) {
    return -1;
  }

  // check fields
  for (int i = 0; i < 3; i++) {
    if (!yaml[key][targets[i]]) {
      return -2;
    }
  }

  // check number of rows
  if (yaml[key]["rows"].as<int>() != 1) {
    return -2;
  }

  return 0;
}

int yamlCheckMatrix(YAML::Node yaml, std::string key) {
  int retval;
  const std::string targets[3] = {"rows", "cols", "data"};

  // check key
  retval = yamlCheckKey(yaml, key);
  if (retval == -1) {
    return -1;
  }

  // check fields
  for (int i = 0; i < 3; i++) {
    if (!yaml[key][targets[i]]) {
      return -2;
    }
  }

  return 0;
}

int yamlBool(YAML::Node yaml, std::string key, bool &x, bool optional) {
  int retval;

  retval = yamlCheckKey(yaml, key);
  YAML_PRIMITIVE_RETVAL(retval);
  x = yaml[key].as<bool>();

  return 0;
}

int yamlInteger(YAML::Node yaml, std::string key, int &x, bool optional) {
  int retval;

  retval = yamlCheckKey(yaml, key);
  YAML_PRIMITIVE_RETVAL(retval);
  x = yaml[key].as<int>();

  return 0;
}

int yamlFloat(YAML::Node yaml, std::string key, float &x, bool optional) {
  int retval;

  retval = yamlCheckKey(yaml, key);
  YAML_PRIMITIVE_RETVAL(retval);
  x = yaml[key].as<float>();

  return 0;
}

int yamlDouble(YAML::Node yaml, std::string key, double &x, bool optional) {
  int retval;

  retval = yamlCheckKey(yaml, key);
  YAML_PRIMITIVE_RETVAL(retval);
  x = yaml[key].as<double>();

  return 0;
}

int yamlString(YAML::Node yaml,
               std::string key,
               std::string &x,
               bool optional) {
  int retval;

  retval = yamlCheckKey(yaml, key);
  YAML_PRIMITIVE_RETVAL(retval);
  x = yaml[key].as<std::string>();

  return 0;
}

int yamlVec2(YAML::Node yaml, std::string key, Vec2 &x, bool optional) {
  int retval;

  // pre-check
  retval = yamlCheckVector(yaml, key);
  YAML_VECTOR_RETVAL(retval);

  // parse vector
  x(0) = yaml[key]["data"][0].as<double>();
  x(1) = yaml[key]["data"][1].as<double>();

  return 0;
}

int yamlVec3(YAML::Node yaml, std::string key, Vec3 &x, bool optional) {
  int retval;

  // pre-check
  retval = yamlCheckVector(yaml, key);
  YAML_VECTOR_RETVAL(retval);

  // parse vector
  x(0) = yaml[key]["data"][0].as<double>();
  x(1) = yaml[key]["data"][1].as<double>();
  x(2) = yaml[key]["data"][2].as<double>();

  return 0;
}

int yamlVec4(YAML::Node yaml, std::string key, Vec4 &x, bool optional) {
  int retval;

  // pre-check
  retval = yamlCheckVector(yaml, key);
  YAML_VECTOR_RETVAL(retval);

  // parse vector
  x(0) = yaml[key]["data"][0].as<double>();
  x(1) = yaml[key]["data"][1].as<double>();
  x(2) = yaml[key]["data"][2].as<double>();
  x(3) = yaml[key]["data"][3].as<double>();

  return 0;
}

int yamlVecX(YAML::Node yaml, std::string key, VecX &x, bool optional) {
  int retval;
  int index;
  int rows;
  int cols;

  // pre-check
  retval = yamlCheckVector(yaml, key);
  YAML_VECTOR_RETVAL(retval);

  // parse vector
  index = 0;
  rows = yaml[key]["rows"].as<int>();
  cols = yaml[key]["cols"].as<int>();
  x.resize(1, cols);

  for (int i = 0; i < yaml[key]["cols"].as<int>(); i++) {
    x(i) = yaml[key]["data"][index].as<double>();
    index++;
  }

  return 0;
}

int yamlMat2(YAML::Node yaml, std::string key, Mat2 &x, bool optional) {
  int retval;
  int index;
  int rows;
  int cols;

  // pre-check
  retval = yamlCheckMatrix(yaml, key);
  YAML_MATRIX_RETVAL(retval);

  // parse matrix
  index = 0;
  rows = yaml[key]["rows"].as<int>();
  cols = yaml[key]["cols"].as<int>();
  x.resize(rows, cols);

  for (int i = 0; i < yaml[key]["rows"].as<int>(); i++) {
    for (int j = 0; j < yaml[key]["cols"].as<int>(); j++) {
      x(i, j) = yaml[key]["data"][index].as<double>();
      index++;
    }
  }

  return 0;
}

int yamlMat3(YAML::Node yaml, std::string key, Mat3 &x, bool optional) {
  int retval;
  int index;
  int rows;
  int cols;

  // pre-check
  retval = yamlCheckMatrix(yaml, key);
  YAML_MATRIX_RETVAL(retval);

  // parse matrix
  index = 0;
  rows = yaml[key]["rows"].as<int>();
  cols = yaml[key]["cols"].as<int>();
  x.resize(rows, cols);

  for (int i = 0; i < yaml[key]["rows"].as<int>(); i++) {
    for (int j = 0; j < yaml[key]["cols"].as<int>(); j++) {
      x(i, j) = yaml[key]["data"][index].as<double>();
      index++;
    }
  }

  return 0;
}

int yamlMat4(YAML::Node yaml, std::string key, Mat4 &x, bool optional) {
  int retval;
  int index;
  int rows;
  int cols;

  // pre-check
  retval = yamlCheckMatrix(yaml, key);
  YAML_MATRIX_RETVAL(retval);

  // parse matrix
  index = 0;
  rows = yaml[key]["rows"].as<int>();
  cols = yaml[key]["cols"].as<int>();
  x.resize(rows, cols);

  for (int i = 0; i < yaml[key]["rows"].as<int>(); i++) {
    for (int j = 0; j < yaml[key]["cols"].as<int>(); j++) {
      x(i, j) = yaml[key]["data"][index].as<double>();
      index++;
    }
  }

  return 0;
}

int yamlMatX(YAML::Node yaml, std::string key, MatX &x, bool optional) {
  int retval;
  int index;
  int rows;
  int cols;

  // pre-check
  retval = yamlCheckMatrix(yaml, key);
  YAML_MATRIX_RETVAL(retval);

  // parse matrix
  index = 0;
  rows = yaml[key]["rows"].as<int>();
  cols = yaml[key]["cols"].as<int>();
  x.resize(rows, cols);

  for (int i = 0; i < yaml[key]["rows"].as<int>(); i++) {
    for (int j = 0; j < yaml[key]["cols"].as<int>(); j++) {
      x(i, j) = yaml[key]["data"][index].as<double>();
      index++;
    }
  }

  return 0;
}

int yamlCvMat(YAML::Node yaml, std::string key, cv::Mat &x, bool optional) {
  int retval;
  int rows;
  int cols;
  int index;
  double value;

  // pre-check
  retval = yamlCheckMatrix(yaml, key);
  YAML_MATRIX_RETVAL(retval);

  // parse matrix
  rows = yaml[key]["rows"].as<int>();
  cols = yaml[key]["cols"].as<int>();
  x = cv::Mat(rows, cols, CV_64F);

  index = 0;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      value = yaml[key]["data"][index].as<double>();
      x.at<double>(i, j) = value;
      index++;
    }
  }
  return 0;
}

ConfigParser::ConfigParser(void) {
  this->configured = false;
  this->loaded = false;
}

void ConfigParser::addParam(enum Type type, std::string key, void *out) {
  ConfigParam param;

  param.type = type;
  param.key = key;

  switch (type) {
    // clang-format off
    case BOOL: param.b = (bool *) out; break;
    case INT: param.i = (int *) out; break;
    case FLOAT: param.f = (float *) out; break;
    case STRING: param.s = (std::string *) out; break;
      // clang-format on
  }

  this->params.push_back(param);
}

int ConfigParser::checkKey(std::string key, bool optional) {
  if (!this->root[key] && optional == false) {
    log_err("Opps [%s] missing in yaml file!", key.c_str());
    return -1;
  } else if (!this->root[key] && optional == true) {
    return 1;
  }

  return 0;
}

int ConfigParser::loadPrimitive(ConfigParam param) {
  int retval;

  // pre-check
  retval = this->checkKey(param.key, param.optional);
  if (retval != 0) {
    return retval;
  }

  // parse
  switch (param.type) {
    case BOOL:
      *param.b = this->root[param.key].as<bool>();
      break;
    case INT:
      *param.i = this->root[param.key].as<int>();
      break;
    case FLOAT:
      *param.f = this->root[param.key].as<float>();
      break;
    case DOUBLE:
      *param.d = this->root[param.key].as<double>();
      break;
    case STRING:
      *param.s = this->root[param.key].as<std::string>();
      break;
  }

  return 0;
}

int ConfigParser::loadArray(ConfigParam param) {
  int retval;
  YAML::Node node;

  // pre-check
  retval = this->checkKey(param.key, param.optional);
  if (retval != 0) {
    return retval;
  }

  // parse
  node = this->root[param.key];
  switch (param.type) {
    case BOOL_ARRAY:
      for (int i = 0; i < node.size(); i++) {
        param.b_array->push_back(node[i].as<bool>());
      }
      break;
    case INT_ARRAY:
      for (int i = 0; i < node.size(); i++) {
        param.i_array->push_back(node[i].as<int>());
      }
      break;
    case FLOAT_ARRAY:
      for (int i = 0; i < node.size(); i++) {
        param.f_array->push_back(node[i].as<float>());
      }
      break;
    case DOUBLE_ARRAY:
      for (int i = 0; i < node.size(); i++) {
        param.d_array->push_back(node[i].as<double>());
      }
      break;
    case STRING_ARRAY:
      for (int i = 0; i < node.size(); i++) {
        param.s_array->push_back(node[i].as<std::string>());
      }
      break;
  }

  return 0;
}

int ConfigParser::loadVector(ConfigParam param) {
  int retval;
  int index;
  int rows;
  int cols;
  YAML::Node node;

  // pre-check
  retval = this->checkKey(param.key, param.optional);
  if (retval != 0) {
    return retval;
  }

  // setup
  Vec2 &vec2 = *param.vec2;
  Vec3 &vec3 = *param.vec3;
  Vec4 &vec4 = *param.vec4;
  VecX &vecx = *param.vecx;

  // parse
  node = this->root[param.key];
  index = 0;
  rows = node["rows"].as<int>();
  cols = node["cols"].as<int>();

  switch (param.type) {
    case VEC2:
      vec2(0) = node["data"][0].as<double>();
      vec2(1) = node["data"][1].as<double>();
      break;
    case VEC3:
      vec3(0) = node["data"][0].as<double>();
      vec3(1) = node["data"][1].as<double>();
      vec3(2) = node["data"][2].as<double>();
      break;
    case VEC4:
      vec4(0) = node["data"][0].as<double>();
      vec4(1) = node["data"][1].as<double>();
      vec4(2) = node["data"][2].as<double>();
      vec4(3) = node["data"][3].as<double>();
      break;
    case VECX:
      vecx = VecX(cols);
      for (int i = 0; i < cols; i++) {
        vecx(i) = node["data"][i].as<double>();
      }
      break;
  }

  return 0;
}

int ConfigParser::loadMatrix(ConfigParam param) {
  int retval;
  int index;
  int rows;
  int cols;
  YAML::Node node;

  // pre-check
  retval = this->checkKey(param.key, param.optional);
  if (retval != 0) {
    return retval;
  }

  // setup
  Mat2 &mat2 = *param.mat2;
  Mat3 &mat3 = *param.mat3;
  Mat4 &mat4 = *param.mat4;
  MatX &matx = *param.matx;

  // parse
  node = this->root[param.key];
  index = 0;
  rows = node["rows"].as<int>();
  cols = node["cols"].as<int>();

  switch (param.type) {
    case MAT2:
      mat2(0, 0) = node["data"][0].as<double>();
      mat2(0, 1) = node["data"][1].as<double>();

      mat2(1, 0) = node["data"][2].as<double>();
      mat2(1, 1) = node["data"][3].as<double>();
      break;
    case MAT3:
      mat3(0, 0) = node["data"][0].as<double>();
      mat3(0, 1) = node["data"][1].as<double>();
      mat3(0, 2) = node["data"][2].as<double>();

      mat3(1, 0) = node["data"][3].as<double>();
      mat3(1, 1) = node["data"][4].as<double>();
      mat3(1, 2) = node["data"][5].as<double>();

      mat3(2, 0) = node["data"][6].as<double>();
      mat3(2, 1) = node["data"][7].as<double>();
      mat3(2, 2) = node["data"][8].as<double>();
      break;
    case MAT4:
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          mat4(i, j) = node["data"][index].as<double>();
          index++;
        }
      }
      break;
    case MATX:
      matx.resize(rows, cols);
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          matx(i, j) = node["data"][index].as<double>();
          index++;
        }
      }
      break;
  }

  return 0;
}

int ConfigParser::load(std::string config_file) {
  int retval;
  ConfigParam param;

  // pre-check
  if (file_exists(config_file) == false) {
    log_err("File not found: %s", config_file.c_str());
    return -1;
  }

  // load and parse file
  this->root = YAML::LoadFile(config_file);
  for (int i = 0; i < params.size(); i++) {
    param = params[i];
  }

  return 0;
}


}  // end of awesomo namespace
