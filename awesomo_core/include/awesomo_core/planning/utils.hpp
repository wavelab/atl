#ifndef __AWESOMO_PLANNING_UTILS_HPP__
#define __AWESOMO_PLANNING_UTILS_HPP__

#include "awesomo_core/utils/utils.hpp"

namespace awesomo {

std::vector<Eigen::Vector2d> bezier_cubic_curve(Eigen::Vector2d p0,
                                                Eigen::Vector2d p1,
                                                Eigen::Vector2d p2,
                                                Eigen::Vector2d p3);

}  // end of awesomo namespace
#endif
