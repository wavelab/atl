#include "awesomo_core/utils/config.hpp"


namespace awesomo {

int yamlIntegerToInteger(YAML::Node yaml) {
  return yaml.as<int>();
}

float yamlFloatToFloat(YAML::Node yaml) {
  return yaml.as<int>();
}

double yamlDoubleToDouble(YAML::Node yaml) {
  return yaml.as<double>();
}

std::string yamlStringToString(YAML::Node yaml) {
  return yaml.as<std::string>();
}

Vec3 yamlVec3ToVec3(YAML::Node yaml) {
  Vec3 out;

  out(0) = yaml[0].as<double>();
  out(1) = yaml[1].as<double>();
  out(2) = yaml[2].as<double>();

  return out;
}

Vec4 yamlVec4ToVec4(YAML::Node yaml) {
  Vec4 out;

  out(0) = yaml[0].as<double>();
  out(1) = yaml[1].as<double>();
  out(2) = yaml[2].as<double>();
  out(3) = yaml[3].as<double>();

  return out;
}

MatX yamlMatXToMatX(YAML::Node yaml) {
  MatX out;
  int index;

  index = 0;
  out.resize(yaml["rows"].as<int>(), yaml["cols"].as<int>());

  for (int i = 0; i < yaml["rows"].as<int>(); i++) {
    for (int j = 0; j < yaml["cols"].as<int>(); j++) {
      out(i, j) = yaml["data"][index].as<double>();
      index++;
    }
  }

  return out;
}

int yamlCheckMatrix(YAML::Node yaml) {
  const std::string targets[3] = {"rows", "cols", "data"};

  // pre-check
  if (yaml == NULL) {
    return -1;
  }

  for (int i = 0; i < 3; i++) {
    if (!yaml[targets[i]]) {
      return -1;
    }
  }

  return 0;
}

cv::Mat yamlMatToCvMat(YAML::Node yaml) {
  int rows;
  int cols;
  int index;
  double value;

  // load matrix
  rows = yaml["rows"].as<int>();
  cols = yaml["cols"].as<int>();
  cv::Mat mat(rows, cols, CV_64F);

  index = 0;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      value = yaml["data"][index].as<double>();
      mat.at<double>(i, j) = value;
      index++;
    }
  }

  return mat;
}

MatX yamlMat2Mat(YAML::Node yaml) {
  int rows;
  int cols;
  int index;
  double value;

  // load matrix
  rows = yaml["rows"].as<int>();
  cols = yaml["cols"].as<int>();
  MatX mat(rows, cols);

  index = 0;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      value = yaml["data"][index].as<double>();
      mat(i, j) = value;
      index++;
    }
  }

  return mat;
}

int yamlSetInteger(YAML::Node yaml, std::string key, int &x) {
  if (yaml[key]) {
    x = yaml[key].as<int>();
  } else {
    log_err("Opps [%s] missing in yaml file\n", key.c_str());
  }

  return 0;
}

int yamlSetDouble(YAML::Node yaml, std::string key, double &x) {
  if (yaml[key]) {
    x = yaml[key].as<double>();
  } else {
    log_err("Opps [%s] missing in yaml file\n", key.c_str());
  }

  return 0;
}

}  // end of awesomo namespace
