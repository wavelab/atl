#ifndef __atl_UTILS_STATS_HPP__
#define __atl_UTILS_STATS_HPP__

#include "atl_core/utils/math.hpp"

namespace atl {

int linreg(std::vector<Eigen::Vector2d> pts, double *m, double *b, double *r);

}  // end of atl namespace
#endif
