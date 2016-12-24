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

}  // end of awesomo namespace
