#ifndef ATL_PLANNING_UTILS_HPP
#define ATL_PLANNING_UTILS_HPP

#include "atl/utils/utils.hpp"

namespace atl {

std::vector<Eigen::Vector2d> bezier_cubic_curve(Eigen::Vector2d p0,
                                                Eigen::Vector2d p1,
                                                Eigen::Vector2d p2,
                                                Eigen::Vector2d p3);

}  // namespace atl
#endif
