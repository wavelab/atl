#ifndef __atl_UTILS_TIME_HPP__
#define __atl_UTILS_TIME_HPP__

#include <time.h>
#include <sys/time.h>

namespace atl {

void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
double time_now(void);

}  // end of atl namespace
#endif
