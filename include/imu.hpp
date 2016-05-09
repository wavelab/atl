#ifndef __IMU_HPP__
#define __IMU_HPP__

#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "navio2/MPU9250.h"
#include "navio2/Util.h"


// CLASSES
class Accelerometer
{
    public:
        float x;
        float y;
        float z;
        float roll;
        float pitch;

        Accelerometer(void);
};

class Gyroscope
{
    public:
        float x;
        float y;
        float z;
        float roll;
        float pitch;

        Gyroscope(void);
};

class Magnetometer
{
    public:
        float x;
        float y;
        float z;
        float roll;
        float pitch;

        Magnetometer(void);
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
        clock_t last_updated;

        IMU(void);
        void read(void);
        void calculateOrientationCF(void);
        void update(void);
        void print(void);
};


#endif
