#ifndef ATL_UTILS_TIME_HPP
#define ATL_UTILS_TIME_HPP

#include <sys/time.h>
#include <time.h>

namespace atl {

void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
double time_now();

}  // namespace atl
#endif
