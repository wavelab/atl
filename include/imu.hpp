#ifndef __IMU_HPP__
#define __IMU_HPP__

#include <iostream>
#include <fstream>

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>

#include <navio2/MPU9250.h>
#include <navio2/LSM9DS1.h>
#include <navio2/Util.h>

#include <yaml-cpp/yaml.h>

#include "ekf.hpp"
#include "util.hpp"


// CONSTANTS
#define IMU_IDLE 0
#define IMU_RUNNING 1
#define IMU_UNIT_TESTING 2

#define ACCELEROMETER_CALIBRATION_INSTRUCTIONS \
    "Acceleration Calibration\n";

#define GYROSCOPE_CALIBRATION_INSTRUCTIONS \
    "Gyroscope Calibration\n";

#define MAGNETOMETER_CALIBRATION_INSTRUCTIONS \
    "Magnetometer Calibration\n" \
    "Move the quadrotor in a figure of 8 for 20 seconds!\n";


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
    void LoadConfiguration(const std::string config_path);
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
    void LoadConfiguration(const std::string config_path);
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
    void LoadConfiguration(const std::string config_path);
    void saveConfiguration(const std::string config_path);
};

class IMU
{
private:
    void printAccelerometerCalibrationInstructions(int i);
    void recordAccelerometerBounds(int i, float *bounds, float *data);
    void printAccelerometerData(char *line, float *data);
    int obtainAccelerometerBounds(float *bounds, char *line);
    int obtainHardIronErrors(const std::string record_path);
    int obtainSoftIronErrors(void);

public:
    int state;

    InertialSensor *mpu9250;
    InertialSensor *lsm9ds1;

    Accelerometer *accel;
    Gyroscope *gyro;
    Magnetometer *mag;

    struct ekf attitude_estimator;

    float roll;
    float pitch;
    float yaw;
    clock_t last_updated;

    IMU(void);
    void initialize(void);
    int update(void);
    int calibrateGyroscope(const std::string config_path);
    int calibrateAccelerometer(const std::string config_path);
    int calibrateMagnetometer(
        const std::string config_path,
        const std::string record_path,
        int test_mode
    );
    int calibrateMagnetometer(
        const std::string config_path,
        const std::string record_path
    );
    void calculateOrientationCF(void);
    void calculateOrientationEKF(void);
    void print(void);
};


#endif
