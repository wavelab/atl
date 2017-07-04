#ifndef ATL_UTILS_TIME_HPP
#define ATL_UTILS_TIME_HPP

#include <time.h>
#include <sys/time.h>

namespace atl {

void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
double time_now(void);

}  // namespace atl
#endif
