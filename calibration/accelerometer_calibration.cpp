#include "imu.hpp"

#define CONFIG_PATH "accelerometer.yaml"


int main(void)
{
    IMU imu;
    imu.initialize();
    imu.calibrateAccelerometer(CONFIG_PATH);
    return 0;
}
