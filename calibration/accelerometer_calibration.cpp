#include <iostream>

#include <yaml-cpp/yaml.h>

#include "imu.hpp"

#define CONFIG_PATH "accelerometer.yaml"
#define ACCELEROMETER_CALIBRATION_INSTRUCTIONS \
    "Acceleration Calibration\n";


int main(void)
{
    IMU imu;

    std::cout << ACCELEROMETER_CALIBRATION_INSTRUCTIONS;
    imu.calibrateAccelerometer(CONFIG_PATH);

    return 0;
}
