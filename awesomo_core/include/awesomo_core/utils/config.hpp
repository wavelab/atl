#ifndef __AWESOMO_UTILS_CONFIG_HPP__
#define __AWESOMO_UTILS_CONFIG_HPP__

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "awesomo_core/utils/math.hpp"


namespace awesomo {

Vec3 yamlVec3ToVec3(YAML::Node in);
Vec4 yamlVec4ToVec4(YAML::Node in);
MatX yamlMatXToMatX(YAML::Node in);
int yamlCheckMatrix(YAML::Node yaml);
cv::Mat yamlMatToCvMat(YAML::Node yaml);

}  // end of awesomo namespace
#endif
