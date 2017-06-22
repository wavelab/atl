#include "atl_core/sensor/MPU6050.hpp"


namespace atl {

MPU6050::MPU6050(void) {

}

int8_t MPU6050::configure(void) {
  int8_t retval;

  /* setup */
  this->i2c = I2C();
  if (this->i2c.setup() != 0) {
    log_err("Failed to initilize I2C!");
  }
  this->i2c.setSlave(MPU6050_ADDRESS);

  /* set intial values */
  this->gyro.offset_x = 0.0f;
  this->gyro.offset_y = 0.0f;
  this->gyro.offset_z = 0.0f;
  this->gyro.pitch = 0.0f;
  this->gyro.roll = 0.0f;

  this->accel.offset_x = 0.0f;
  this->accel.offset_y = 0.0f;
  this->accel.offset_z = 0.0f;
  this->accel.pitch = 0.0f;
  this->accel.roll = 0.0f;

  this->pitch_offset = 0.0f;
  this->roll_offset = 0.0f;

  this->pitch = 0.0f;
  this->roll = 0.0f;
  this->temperature = 0.0f;

  this->last_updated = clock();
  this->sample_rate = -1.0;
  this->dplf_config = 0;

  /* set dplf */
  this->setDPLFConfig(6);
  retval = this->getDPLFConfig();
  if (retval > 7 || retval < 0) {
    return -1;
  } else {
    this->dplf_config = retval;
    log_info("dplf config: %d", this->dplf_config);
  }

  /* set power management register */
  this->i2c.writeByte(MPU6050_RA_PWR_MGMT_1, 0x00);

  /* get gyro range */
  this->setGyroRange(0);
  retval = this->getGyroRange();
  if (retval == 0) {
    this->gyro.sensitivity = 131.0;
  } else if (retval == 1) {
    this->gyro.sensitivity = 65.5;
  } else if (retval == 2) {
    this->gyro.sensitivity = 32.8;
  } else if (retval == 3) {
    this->gyro.sensitivity = 16.4;
  } else {
    return -2;
  }

  /* get accel range */
  this->setAccelRange(0);
  retval = this->getAccelRange();
  if (retval == 0) {
    this->accel.sensitivity = 16384.0;
  } else if (retval == 1) {
    this->accel.sensitivity = 8192.0;
  } else if (retval == 2) {
    this->accel.sensitivity = 4096.0;
  } else if (retval == 3) {
    this->accel.sensitivity = 2048.0;
  } else {
    return -3;
  }

  /* get sample rate */
  this->sample_rate = this->getSampleRate();

  /* calibrate mpu6050 */
  // this->calibrate();

  return 0;
}

int8_t MPU6050::ping(void) {
  char buf;

  /* print mpu6050 address */
  this->i2c.setSlave(MPU6050_ADDRESS);
  this->i2c.readByte(MPU6050_RA_WHO_AM_I, &buf);
  printf("MPU6050 ADDRESS: 0x%02X\n", buf);

  return 0;
}

void MPU6050::accelerometerCalcAngle(void) {
  float x;
  float y;
  float z;

  /* setup */
  x = this->accel.x;
  y = this->accel.y;
  z = this->accel.z;

  /* calculate pitch and roll */
  this->accel.pitch = (atan(x / sqrt(pow(y, 2) + pow(z, 2)))) * 180 / M_PI;
  this->accel.roll = (atan(y / sqrt(pow(x, 2) + pow(z, 2)))) * 180 / M_PI;
}

void MPU6050::gyroscopeCalcAngle(float dt) {
  this->gyro.roll = (this->gyro.x * dt) + this->roll;
  this->gyro.pitch = (this->gyro.y * dt) + this->pitch;
}

int8_t MPU6050::getData(void) {
  char raw_data[14];
  int8_t raw_temp;
  float dt;
  clock_t time_now;
  int retval;

  /* read this data */
  memset(raw_data, '\0', 14);
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.readBytes(MPU6050_RA_ACCEL_XOUT_H, raw_data, 14);
  if (retval != 0) {
    return -1;
  }

  /* accelerometer */
  this->accel.raw_x = (raw_data[0] << 8) | (raw_data[1]);
  this->accel.raw_y = (raw_data[2] << 8) | (raw_data[3]);
  this->accel.raw_z = (raw_data[4] << 8) | (raw_data[5]);

  this->accel.raw_x -= this->accel.offset_x;
  this->accel.raw_y -= this->accel.offset_y;
  this->accel.raw_z -= this->accel.offset_z;

  this->accel.x = this->accel.raw_x / this->accel.sensitivity;
  this->accel.y = this->accel.raw_y / this->accel.sensitivity;
  this->accel.z = this->accel.raw_z / this->accel.sensitivity;

  /* temperature */
  raw_temp = (raw_data[6] << 8) | (raw_data[7]);
  this->temperature = raw_temp / 340.0 + 36.53;

  /* gyroscope */
  this->gyro.raw_x = (raw_data[8] << 8) | (raw_data[9]);
  this->gyro.raw_y = (raw_data[10] << 8) | (raw_data[11]);
  this->gyro.raw_z = (raw_data[12] << 8) | (raw_data[13]);

  this->gyro.raw_x -= this->gyro.offset_x;
  this->gyro.raw_y -= this->gyro.offset_y;
  this->gyro.raw_z -= this->gyro.offset_z;

  this->gyro.x = this->gyro.raw_x / this->gyro.sensitivity;
  this->gyro.y = this->gyro.raw_y / this->gyro.sensitivity;
  this->gyro.z = this->gyro.raw_z / this->gyro.sensitivity;

  /* calculate dt */
  time_now = clock();
  dt = ((double) time_now - this->last_updated) / CLOCKS_PER_SEC;

  /* complimentary filter */
  this->accelerometerCalcAngle();
  this->pitch = (0.90 * this->gyro.pitch) + (0.1 * this->accel.pitch);
  this->roll = (0.90 * this->gyro.roll) + (0.1 * this->accel.roll);
  this->gyroscopeCalcAngle(dt);

  /* offset pitch and roll */
  this->pitch += this->pitch_offset;
  this->roll += this->roll_offset;

  /* set last_updated */
  this->last_updated = clock();

  return 0;
}

int8_t MPU6050::calibrate(void) {
  int16_t i;

  /* let it stablize for a while first */
  log_info("calibrating mpu6050");
  for (i = 0; i < 50; i++) {
    this->getData();
  }

  /* calculate offset */
  for (i = 0; i < 50; i++) {
    this->getData();

    this->accel.offset_x += this->accel.raw_x;
    this->accel.offset_y += this->accel.raw_y;
    this->accel.offset_z += this->accel.raw_z;

    this->accel.offset_x = this->accel.offset_x / 2.0;
    this->accel.offset_y = this->accel.offset_y / 2.0;
    this->accel.offset_z = this->accel.offset_z / 2.0;

    this->gyro.offset_x += this->gyro.raw_x;
    this->gyro.offset_y += this->gyro.raw_y;
    this->gyro.offset_z += this->gyro.raw_z;

    this->gyro.offset_x = this->gyro.offset_x / 2.0;
    this->gyro.offset_y = this->gyro.offset_y / 2.0;
    this->gyro.offset_z = this->gyro.offset_z / 2.0;
  }

  return 0;
}

void MPU6050::print(void) {
  printf("gyro_x: %f\n", this->gyro.x);
  printf("gyro_y: %f\n", this->gyro.y);
  printf("gyro_z: %f\n", this->gyro.z);

  printf("accel x: %f\n", this->accel.x);
  printf("accel y: %f\n", this->accel.y);
  printf("accel z: %f\n", this->accel.z);

  printf("\n");
  printf("accel pitch: %f\n", this->accel.pitch);
  printf("accel roll: %f\n", this->accel.roll);
  printf("\n");
  printf("gyro pitch: %f\n", this->gyro.pitch);
  printf("gyro roll: %f\n", this->gyro.roll);
  printf("\n");

  printf("temp: %f\n", this->temperature);
  printf("\n");
  printf("\n");
}

int8_t MPU6050::setDPLFConfig(int8_t setting) {
  /*
     DPLF_CFG    Accelerometer
     ----------------------------------------
     Bandwidth(Hz) | Delay(ms)
     0           260             0
     1           184             2.0
     2           94              3.0
     3           44              4.9
     4           21              8.5
     5           10              13.8
     6           5               19.0
     7           RESERVED        RESERVED


     DPLF_CFG    Gyroscope
     ----------------------------------------------
     Bandwidth(Hz) | Delay(ms) | Fs(kHz)
     0           256             0.98        8
     1           188             1.9         1
     2           98              2.8         1
     3           42              4.8         1
     4           20              8.3         1
     5           10              13.4        1
     6           5               18.5        1
     7           RESERVED        RESERVED    8
  */
  int retval;

  /* check setting range */
  if (setting > 7 || setting < 0) {
    return -2;
  }

  /* set DPLF */
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.writeByte(MPU6050_RA_CONFIG, (char) setting);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int8_t MPU6050::getDPLFConfig(void) {
  char data[1];
  int retval;

  /* get dplf config */
  data[0] = 0x00;
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.readBytes(MPU6050_RA_CONFIG, data, 1);
  if (retval != 0) {
    return -1;
  }

  log_info("GOT DPLF: %d", data[0]);
  data[0] = data[0] & 0b00000111;

  return data[0];
}

int8_t MPU6050::setSampleRateDiv(int8_t div) {
  int retval;

  /* set sample rate divider */
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.writeByte(MPU6050_RA_SMPLRT_DIV, div);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int8_t MPU6050::getSampleRateDiv(void) {
  char data;
  int retval;

  /* get sample rate */
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.readByte(MPU6050_RA_SMPLRT_DIV, &data);
  if (retval != 0) {
    return -1;
  }

  return data;
}

int16_t MPU6050::getSampleRate(void) {
  uint8_t smplrt_div;
  uint8_t dlpf_cfg;
  uint16_t sample_divider;
  uint16_t gyro_rate;

  /* get sample rate divider */
  smplrt_div = this->getSampleRateDiv();
  if (smplrt_div != -1 || smplrt_div != -2) {
    sample_divider = (float) smplrt_div;
  } else {
    return -1;
  }

  /* get gyro sample rate */
  dlpf_cfg = this->getSampleRateDiv();
  if (dlpf_cfg == 0 || dlpf_cfg == 7) {
    gyro_rate = 8000;
  } else if (dlpf_cfg >= 1 || dlpf_cfg <= 6) {
    gyro_rate = 1000;
  } else {
    return -2;
  }

  /* calculate sample rate */
  return gyro_rate / (1 + sample_divider);
}

int8_t MPU6050::setGyroRange(int8_t range) {
  char data;
  uint8_t retval;

  /* pre-check */
  if (range > 3 || range < 0) {
    return -2;
  }

  /* set sample rate */
  data = range << 3;
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.writeByte(MPU6050_RA_GYRO_CONFIG, data);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int8_t MPU6050::getGyroRange(void) {
  char data;
  int retval;

  /* get gyro config */
  data = 0x00;
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.readByte(MPU6050_RA_GYRO_CONFIG, &data);
  if (retval != 0) {
    return -1;
  }

  /* get gyro range bytes */
  data = (data >> 3) & 0b00000011;

  return data;
}

int8_t MPU6050::setAccelRange(int8_t range) {
  char data;
  uint8_t retval;

  /* pre-check */
  if (range > 3 || range < 0) {
    return -2;
  }

  /* set sample rate */
  data = range << 3;
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.writeByte(MPU6050_RA_ACCEL_CONFIG, data);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int8_t MPU6050::getAccelRange(void) {
  char data;
  uint8_t retval;

  /* get accel config */
  data = 0x00;
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.readByte(MPU6050_RA_ACCEL_CONFIG, &data);
  if (retval != 0) {
    return -1;
  }

  /* get accel range bytes */
  data = (data >> 3) & 0b00000011;

  return data;
}

void MPU6050::info(void) {
  printf("gyro sensitivity: %f\n", this->gyro.sensitivity);
  printf("gyro offset_x: %f\n", this->gyro.offset_x);
  printf("gyro offset_y: %f\n", this->gyro.offset_y);
  printf("gyro offset_z: %f\n", this->gyro.offset_z);
  printf("\n");
  printf("accel sensitivity: %f\n", this->accel.sensitivity);
  printf("accel offset_x: %f\n", this->accel.offset_x);
  printf("accel offset_y: %f\n", this->accel.offset_y);
  printf("accel offset_z: %f\n", this->accel.offset_z);
  printf("\n");
  printf("sample rate: %f\n", this->sample_rate);
  printf("\n");
}

void MPU6050::recordHeader(FILE *output_file) {
  fprintf(output_file, "gyro.x,");
  fprintf(output_file, "gyro.y,");
  fprintf(output_file, "gyro.z,");
  fprintf(output_file, "gyro.pitch,");
  fprintf(output_file, "gyro.roll,");

  fprintf(output_file, "accel.x,");
  fprintf(output_file, "accel.y,");
  fprintf(output_file, "accel.z,");
  fprintf(output_file, "accel.pitch,");
  fprintf(output_file, "accel.roll,");

  fprintf(output_file, "pitch,");
  fprintf(output_file, "roll\n");
}

void MPU6050::recordData(FILE *output_file)
{
  fprintf(output_file, "%f,", this->gyro.x);
  fprintf(output_file, "%f,", this->gyro.y);
  fprintf(output_file, "%f,", this->gyro.z);
  fprintf(output_file, "%f,", this->gyro.pitch);
  fprintf(output_file, "%f,", this->gyro.roll);

  fprintf(output_file, "%f,", this->accel.x);
  fprintf(output_file, "%f,", this->accel.y);
  fprintf(output_file, "%f,", this->accel.z);
  fprintf(output_file, "%f,", this->accel.pitch);
  fprintf(output_file, "%f,", this->accel.roll);

  fprintf(output_file, "%f,", this->pitch);
  fprintf(output_file, "%f\n", this->roll);
}

int8_t MPU6050::record(std::string output_path, int nb_samples) {
  int i;
  int8_t retval;
  FILE *output_file;

  /* setup */
  output_file = fopen(output_path.c_str(), "w");
  this->recordHeader(output_file);

  /* record */
  for (i = 0; i < nb_samples; i++) {
    /* get data */
    retval = this->getData();
    if (retval == -1) {
      log_err("failed to obtain data from MPU6050!");
      return -1;
    }

    /* record data */
    this->recordData(output_file);
    if (retval == -1) {
      log_err("failed to record MPU6050 data!");
      return -1;
    }
  }

  /* clean up */
  fclose(output_file);

  return 0;
}

}  // end of atl namespace
