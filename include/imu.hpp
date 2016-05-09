#ifndef __IMU_HPP__
#define __IMU_HPP__

#include <stdlib.h>
#include <sys/timeb.h>

#include "navio2/MPU9250.h"
#include "navio2/Util.h"


// CLASSES
class Accelerometer
{
    public:
        Accelerometer(void);
        float x;
        float y;
        float z;
        float roll;
        float pitch;
};

class Gyroscope
{
    public:
        Gyroscope(void);
        float x;
        float y;
        float z;
        float roll;
        float pitch;
};

class Magnetometer
{
    public:
        Magnetometer(void);
        float x;
        float y;
        float z;
        float roll;
        float pitch;
};

class IMU
{
    public:
        InertialSensor *mpu9250;
        Accelerometer *accel_data;
        Gyroscope *gyro_data;
        Magnetometer *mag_data;
        float roll;
        float pitch;
        struct timeb last_updated;

        IMU(void);
        void read(void);
        void update(void);
        void print(void);
};


#endif
