#ifndef ATL_UTILS_LOG_HPP
#define ATL_UTILS_LOG_HPP

#define __FILENAME__ \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_ERROR(M, ...)      \
  fprintf(                     \
    stderr,                    \
    "[ERROR] [%s:%d] " M "\n", \
    __FILENAME__,              \
    __LINE__,                  \
    ##__VA_ARGS__)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)

#endif
