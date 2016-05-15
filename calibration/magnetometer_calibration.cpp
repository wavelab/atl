#include "imu.hpp"

#define CONFIG_PATH "magnetometer.yaml"
#define RECORD_PATH "magnetometer.dat"


int main(void)
{
    IMU imu;
    imu.calibrateMagnetometer(CONFIG_PATH, RECORD_PATH);
    return 0;
}
