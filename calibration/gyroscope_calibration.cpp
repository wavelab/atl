#include <iostream>

#include <yaml-cpp/yaml.h>

#include "imu.hpp"

#define CONFIG_PATH "gyroscope.yaml"
#define GYROSCOPE_CALIBRATION_INSTRUCTIONS \
    "Gyroscope Calibration\n";


int main(void)
{
    IMU imu;

    std::cout << GYROSCOPE_CALIBRATION_INSTRUCTIONS;
    imu.calibrateGyroscope(CONFIG_PATH);

    return 0;
}
