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

}  // end of awesomo namespace
