#ifndef ATL_SENSOR_I2C_HPP
#define ATL_SENSOR_I2C_HPP

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

namespace atl {

// ERROR MESSAGES
#define I2C_INIT_FAILED "Failed to initialize I2C!"

// DEFINES
#define I2C_BUF_MAX 1024

class I2C {
public:
  int fd;

  I2C(void);
  ~I2C(void);
  int setup(void);
  int setSlave(char slave_addr);
  int readBytes(char reg_addr, char *data, size_t length);
  int readByte(char reg_addr, char *data);
  int writeByte(char reg_addr, char byte);
  int writeRawByte(char byte);
  int writeBytes(char reg_addr, char *data, size_t length);
};

}  // namespace atl
#endif
