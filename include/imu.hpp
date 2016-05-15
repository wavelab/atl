#ifndef __IMU_HPP__
#define __IMU_HPP__

#include <iostream>
#include <fstream>

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <navio2/MPU9250.h>
#include <navio2/LSM9DS1.h>
#include <navio2/Util.h>

#include <yaml-cpp/yaml.h>

#include "util.hpp"


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
        void saveConfiguration(const std::string config_path);
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
        void saveConfiguration(const std::string config_path);
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

        float x_min;
        float x_max;

        float y_min;
        float y_max;

        float z_min;
        float z_max;

        float scale_x;
        float scale_y;
        float scale_z;

        float bearing;

        Magnetometer(void);
        void saveConfiguration(const std::string config_path);
};

class IMU
{
    private:
        void obtainHardIronErrors(const std::string record_path);
        void obtainSoftIronErrors(void);

    public:
        InertialSensor *mpu9250;
        InertialSensor *lsm9ds1;

        Accelerometer *accel;
        Gyroscope *gyro;
        Magnetometer *mag;

        float roll;
        float pitch;
        float yaw;
        clock_t last_updated;

        IMU(void);
        void calibrateGyroscope(const std::string config_path);
        void calibrateAccelerometer(const std::string config_path);
        void calibrateMagnetometer(
            const std::string config_path,
            const std::string record_path
        );
        void calculateOrientationCF(void);
        void update(void);
        void print(void);
};


#endif
