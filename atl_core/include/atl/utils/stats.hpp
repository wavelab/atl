#ifndef ATL_UTILS_STATS_HPP
#define ATL_UTILS_STATS_HPP

#include "atl/utils/math.hpp"

namespace atl {

int linreg(std::vector<Eigen::Vector2d> pts, double *m, double *b, double *r);

}  // namespace atl
#endif
