#ifndef __AWESOMO_UTILS_UTILS_HPP__
#define __AWESOMO_UTILS_UTILS_HPP__

#include "awesomo_core/utils/config.hpp"
#include "awesomo_core/utils/data.hpp"
#include "awesomo_core/utils/io.hpp"
#include "awesomo_core/utils/math.hpp"
#include "awesomo_core/utils/stats.hpp"
#include "awesomo_core/utils/time.hpp"

#define __FILENAME__ \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define log_err(M, ...)              \
  fprintf(stderr,                    \
          "[ERROR] [%s:%d] " M "\n", \
          __FILENAME__,              \
          __LINE__,                  \
          ##__VA_ARGS__)

#define log_info(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)

#endif
