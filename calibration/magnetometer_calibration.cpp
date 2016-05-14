#include <iostream>

#include <yaml-cpp/yaml.h>

#include "imu.hpp"

#define CONFIG_PATH "magnetometer.yaml"
#define RECORD_PATH "magnetometer.dat"
#define MAGNETOMETER_CALIBRATION_INSTRUCTIONS \
    "Magnetometer Calibration\n" \
    "Move the quadrotor in a figure of 8 for 20 seconds!\n";


int main(void)
{
    IMU imu;

    std::cout << MAGNETOMETER_CALIBRATION_INSTRUCTIONS;
    imu.calibrateMagnetometer(CONFIG_PATH, RECORD_PATH);

    return 0;
}
