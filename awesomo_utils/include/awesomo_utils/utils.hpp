#ifndef __AWESOMO_UTILS_UTIL_HPP__
#define __AWESOMO_UTILS_UTIL_HPP__

#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "awesomo_utils/math.hpp"

namespace awesomo {

void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);

}  // end of awesomo namespace
#endif
