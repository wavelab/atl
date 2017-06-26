#include "atl/sensor/i2c.hpp"


namespace atl {

I2C::I2C(void) {
  this->fd = -1;
}

I2C::~I2C(void) {
  close(this->fd);
}

int I2C::setup(void) {
  int fd;
  int adapter_nr;
  char filename[20];

  /* setup */
  adapter_nr = 1;  /* probably dynamically determined */
  memset(filename, '\0', sizeof(char) * 20);
  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);

  /* open i2c connection */
  fd = open(filename, O_RDWR);
  if (fd < 0) {
    return -1;
  } else {
    this->fd = fd;
  }

  return 0;
}

int I2C::setSlave(char slave_addr) {
  return ioctl(this->fd, I2C_SLAVE , slave_addr);
}

int I2C::readByte(char reg_addr, char *data) {
  char buf[1];

  buf[0] = reg_addr;
  if (write(this->fd, buf, 1) != 1) {
    return -1;
  }

  if (read(this->fd, data, 1) != 1) {
    return -2;
  }

  return 0;
}

int I2C::readBytes(char reg_addr, char *data, size_t length) {
  char buf[1];

  buf[0] = reg_addr;
  if (write(this->fd, buf, 1) != 1) {
    return -1;
  }

  if (read(this->fd, data, length) != (int) length) {
    return -2;
  }

  return 0;
}

int I2C::writeByte(char reg_addr, char byte) {
  char buf[2];

  buf[0] = reg_addr;
  buf[1] = byte;
  if (write(this->fd, buf, 2) != 1) {
    return -1;
  }

  return 0;
}

int I2C::writeRawByte(char byte) {
  if (write(this->fd, &byte, 1) != 1) {
    return -1;
  }

  return 0;
}

int I2C::writeBytes(char reg_addr, char *data, size_t length) {
  int i;
  char buf[I2C_BUF_MAX];

  /* create buf */
  memset(buf, '\0', sizeof(char) * I2C_BUF_MAX);
  buf[0] = reg_addr;
  for (i = 1; i < (int) length + 1; i++) {
    buf[i] = data[i];
  }

  /* write bytes */
  if (write(this->fd, buf, length + 1) != 1) {
    return -1;
  }

  return 0;
}

}  // end of atl namespace
