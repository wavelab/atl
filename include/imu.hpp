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
        float offset_x;
        float offset_y;
        float offset_z;
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
        float offset_x;
        float offset_y;
        float offset_z;
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
        float offset_x;
        float offset_y;
        float offset_z;
        float roll;
        float pitch;

        Magnetometer(void);
};

class AHRS
{
    private:
        float q0, q1, q2, q3;
        float gyroOffset[3];
        float twoKi;
        float twoKp;
        float integralFBx, integralFBy, integralFBz;

    public:
        AHRS(float q0 = 1, float q1 = 0, float q2 = 0, float q3 = 0)
        :    q0(q0), q1(q1), q2(q2), q3(q3), twoKi(0), twoKp(2)  {;}

        void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);
        void updateIMU(float ax, float ay, float az, float gx, float gy, float gz, float dt);
        void setGyroOffset(float offsetX, float offsetY, float offsetZ);
        void getEuler(float* roll, float* pitch, float* yaw);
        float invSqrt(float x);
        float getW();
        float getX();
        float getY();
        float getZ();
};

class IMU
{
    public:
        InertialSensor *mpu9250;
        Accelerometer *accel_data;
        Gyroscope *gyro_data;
        Magnetometer *mag_data;
        AHRS *sensor_fusion;
        float roll;
        float pitch;
        float yaw;
        clock_t last_updated;

        IMU(void);
        void calibrateGyroscope(void);
        void calculateOrientationCF(void);
        void update(void);
        void print(void);
};


#endif
