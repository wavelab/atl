#ifndef __AWESOMO_UTILS_STATS_HPP__
#define __AWESOMO_UTILS_STATS_HPP__

#include "awesomo_core/utils/math.hpp"

namespace awesomo {

int linreg(std::vector<Eigen::Vector2d> pts, double *m, double *b, double *r);

}  // end of awesomo namespace
#endif
