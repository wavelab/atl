#ifndef __atl_PLANNING_UTILS_HPP__
#define __atl_PLANNING_UTILS_HPP__

#include "atl_core/utils/utils.hpp"

namespace atl {

std::vector<Eigen::Vector2d> bezier_cubic_curve(Eigen::Vector2d p0,
                                                Eigen::Vector2d p1,
                                                Eigen::Vector2d p2,
                                                Eigen::Vector2d p3);

}  // end of atl namespace
#endif
