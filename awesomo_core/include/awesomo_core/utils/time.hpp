#ifndef __AWESOMO_UTILS_TIME_HPP__
#define __AWESOMO_UTILS_TIME_HPP__

#include <time.h>
#include <sys/time.h>

namespace awesomo {

void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
double time_now(void);

}  // end of awesomo namespace
#endif
