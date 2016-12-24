#ifndef __AWESOMO_UTILS_CONFIG_HPP__
#define __AWESOMO_UTILS_CONFIG_HPP__

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "awesomo_core/utils/math.hpp"
#include "awesomo_core/utils/logging.hpp"


namespace awesomo {

int yamlIntegerToInteger(YAML::Node yaml);
float yamlFloatToFloat(YAML::Node yaml);
double yamlDoubleToDouble(YAML::Node yaml);
std::string yamlStringToString(YAML::Node yaml);
Vec3 yamlVec3ToVec3(YAML::Node yaml);
Vec4 yamlVec4ToVec4(YAML::Node yaml);
MatX yamlMatXToMatX(YAML::Node yaml);
int yamlCheckMatrix(YAML::Node yaml);
cv::Mat yamlMatToCvMat(YAML::Node yaml);
int yamlSetInteger(YAML::Node yaml, std::string key, int &x);
int yamlSetDouble(YAML::Node yaml, std::string key, double &x);

}  // end of awesomo namespace
#endif
