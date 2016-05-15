#include "imu.hpp"

#define CONFIG_PATH "gyroscope.yaml"


int main(void)
{
    IMU imu;
    imu.initialize();
    imu.calibrateGyroscope(CONFIG_PATH);
    return 0;
}
