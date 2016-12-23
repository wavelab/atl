#include "awesomo_core/utils/config.hpp"


namespace awesomo {

Vec3 yamlVec3ToVec3(YAML::Node in) {
  Vec3 out;

  out(0) = in[0].as<double>();
  out(1) = in[1].as<double>();
  out(2) = in[2].as<double>();

  return out;
}

Vec4 yamlVec4ToVec4(YAML::Node in) {
  Vec4 out;

  out(0) = in[0].as<double>();
  out(1) = in[1].as<double>();
  out(2) = in[2].as<double>();
  out(3) = in[3].as<double>();

  return out;
}

MatX yamlMatXToMatX(YAML::Node in) {
  MatX out;
  int index;

  index = 0;
  out.resize(in["rows"].as<int>(), in["cols"].as<int>());

  for (int i = 0; i < in["rows"].as<int>(); i++) {
    for (int j = 0; j < in["cols"].as<int>(); j++) {
      out(i, j) = in["data"][index].as<double>();
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

cv::Mat yamlMatToCvMat(YAML::Node matrix_yaml) {
  int rows;
  int cols;
  int index;
  double value;

  // load matrix
  rows = matrix_yaml["rows"].as<int>();
  cols = matrix_yaml["cols"].as<int>();
  cv::Mat mat(rows, cols, CV_64F);

  index = 0;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      value = matrix_yaml["data"][index].as<double>();
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

}  // end of awesomo namespace
