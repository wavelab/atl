#ifndef __AWESOMO_UTILS_CONFIG_HPP__
#define __AWESOMO_UTILS_CONFIG_HPP__

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/math.hpp"


namespace awesomo {

Vec3 yamlVec3ToVec3(YAML::Node in);
Vec4 yamlVec4ToVec4(YAML::Node in);
MatX yamlMatXToMatX(YAML::Node in);

}  // end of awesomo namespace
#endif
