#include <iostream>

#include <yaml-cpp/yaml.h>

#include "imu.hpp"

#define CONFIG_PATH "gyroscope.yaml"
#define GYROSCOPE_CALIBRATION_INSTRUCTIONS \
    "Gyroscope Calibration\n";


int main(void)
{
    int loop;
    char c;
    IMU imu;

    // setup
    loop = 1;
    std::cout << GYROSCOPE_CALIBRATION_INSTRUCTIONS;

    // calibrate gyroscope
    imu.calibrateGyroscope(CONFIG_PATH);

    return 0;
}
