#include <iostream>

#include <yaml-cpp/yaml.h>

#include "imu.hpp"

#define CONFIG_PATH "accelerometer.yaml"
#define ACCELEROMETER_CALIBRATION_INSTRUCTIONS \
    "Acceleration Calibration\n";


int main(void)
{
    int loop;
    char c;
    IMU imu;

    // setup
    loop = 1;
    std::cout << ACCELEROMETER_CALIBRATION_INSTRUCTIONS;

    // calibrate magnetometer
    imu.calibrateAccelerometer(CONFIG_PATH);

    return 0;
}
