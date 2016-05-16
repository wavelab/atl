#include "munit.h"
#include "imu.hpp"
#include "util.hpp"


// CONSTANTS
#define GYRO_CONFIG_FILE "./tests/data/gyroscope.yaml"
#define ACCEL_CONFIG_FILE "./tests/data/accelerometer.yaml"
#define MAG_CONFIG_FILE "./tests/data/magnetometer.yaml"
#define MAG_RECORD_FILE "./tests/data/magnetometer.dat"


// TESTS
int testAccelerometerSaveLoadConfiguration(void);
int testGyroscopeSaveLoadConfiguration(void);
int testMagnetometerSaveLoadConfiguration(void);
int testIMUUpdate(void);
int testIMUCalibrateGyroscope(void);
int testIMUCalibrateAccelerometer(void);
int testIMUCalibrateMagnetometer(void);


int testAccelerometerSaveLoadConfiguration(void)
{
    Accelerometer accel;

    // test save configuration
    accel.offset_x = 1.0;
    accel.offset_y = 2.0;
    accel.offset_z = 3.0;
    accel.saveConfiguration(ACCEL_CONFIG_FILE);

    // test load configuration
    accel.offset_x = 0.0;
    accel.offset_y = 0.0;
    accel.offset_z = 0.0;
    accel.loadConfiguration(ACCEL_CONFIG_FILE);

    // assert
    mu_check(fltcmp(accel.offset_x, 1.0) == 0);
    mu_check(fltcmp(accel.offset_y, 2.0) == 0);
    mu_check(fltcmp(accel.offset_z, 3.0) == 0);

    return 0;
}

int testGyroscopeSaveLoadConfiguration(void)
{
    Gyroscope gyro;

    // test save configuration
    gyro.offset_x = 1.0;
    gyro.offset_y = 2.0;
    gyro.offset_z = 3.0;
    gyro.saveConfiguration(GYRO_CONFIG_FILE);

    // test load configuration
    gyro.offset_x = 0.0;
    gyro.offset_y = 0.0;
    gyro.offset_z = 0.0;
    gyro.loadConfiguration(GYRO_CONFIG_FILE);

    // assert
    mu_check(fltcmp(gyro.offset_x, 1.0) == 0);
    mu_check(fltcmp(gyro.offset_y, 2.0) == 0);
    mu_check(fltcmp(gyro.offset_z, 3.0) == 0);

    return 0;
}

int testMagnetometerSaveLoadConfiguration(void)
{
    Magnetometer mag;

    // test save configuration
    mag.offset_x = 1.0;
    mag.offset_y = 2.0;
    mag.offset_z = 3.0;
    mag.scale_x = 4.0;
    mag.scale_y = 5.0;
    mag.scale_z = 6.0;
    mag.saveConfiguration(MAG_CONFIG_FILE);

    // test load configuration
    mag.offset_x = 0.0;
    mag.offset_y = 0.0;
    mag.offset_z = 0.0;
    mag.scale_x = 0.0;
    mag.scale_y = 0.0;
    mag.scale_z = 0.0;
    mag.loadConfiguration(MAG_CONFIG_FILE);

    // assert
    mu_check(fltcmp(mag.offset_x, 1.0) == 0);
    mu_check(fltcmp(mag.offset_y, 2.0) == 0);
    mu_check(fltcmp(mag.offset_z, 3.0) == 0);
    mu_check(fltcmp(mag.scale_x, 4.0) == 0);
    mu_check(fltcmp(mag.scale_y, 5.0) == 0);
    mu_check(fltcmp(mag.scale_z, 6.0) == 0);

    return 0;
}

int testIMUUpdate(void)
{
    IMU imu;

    // setup
    imu.state = IMU_UNIT_TESTING;
    imu.initialize();

    imu.gyro->x = 0.0f;
    imu.gyro->y = 0.0f;
    imu.gyro->z = 0.0f;

    imu.accel->x = 0.0f;
    imu.accel->y = 0.0f;
    imu.accel->z = 0.0f;

    imu.mag->x = 0.0f;
    imu.mag->y = 0.0f;
    imu.mag->z = 0.0f;

    // test imu update
    imu.update();
    mu_check(fltcmp(imu.accel->x, 0.0f) == 0);
    mu_check(fltcmp(imu.accel->y, 0.0f) == 0);
    mu_check(fltcmp(imu.accel->z, 0.0f) == 0);

    mu_check(fltcmp(imu.gyro->x, 0.0f) == 0);
    mu_check(fltcmp(imu.gyro->y, 0.0f) == 0);
    mu_check(fltcmp(imu.gyro->z, 0.0f) == 0);

    mu_check(fltcmp(imu.mag->x, 0.0f) == 0);
    mu_check(fltcmp(imu.mag->y, 0.0f) == 0);
    mu_check(fltcmp(imu.mag->z, 0.0f) == 0);

	return 0;
}

int testIMUCalibrateGyroscope(void)
{
    IMU imu;

    imu.state = IMU_UNIT_TESTING;
    imu.calibrateGyroscope(GYRO_CONFIG_FILE);

    mu_check(fltcmp(imu.gyro->offset_x, 1.0) == 0);
    mu_check(fltcmp(imu.gyro->offset_y, 2.0) == 0);
    mu_check(fltcmp(imu.gyro->offset_z, 3.0) == 0);

    return 0;
}

int testIMUCalibrateAccelerometer(void)
{
    IMU imu;

    imu.state = IMU_UNIT_TESTING;
    imu.calibrateAccelerometer(ACCEL_CONFIG_FILE);

    mu_check(fltcmp(imu.accel->offset_x, 2.0) == 0);
    mu_check(fltcmp(imu.accel->offset_y, 2.0) == 0);
    mu_check(fltcmp(imu.accel->offset_z, 2.0) == 0);

    return 0;
}

int testIMUCalibrateMagnetometer(void)
{
    IMU imu;

    imu.state = IMU_UNIT_TESTING;
    imu.calibrateMagnetometer(MAG_CONFIG_FILE, MAG_RECORD_FILE);

    mu_check(fltcmp(imu.mag->offset_x, 0.5) == 0);
    mu_check(fltcmp(imu.mag->offset_y, 1.0) == 0);
    mu_check(fltcmp(imu.mag->offset_z, 1.5) == 0);

    mu_check(fltcmp(imu.mag->scale_x, 2.0) == 0);
    mu_check(fltcmp(imu.mag->scale_y, 1.0) == 0);
    mu_check(fltcmp(imu.mag->scale_z, 0.666667) == 0);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testAccelerometerSaveLoadConfiguration);
    mu_add_test(testGyroscopeSaveLoadConfiguration);
    mu_add_test(testMagnetometerSaveLoadConfiguration);
    mu_add_test(testIMUUpdate);
    mu_add_test(testIMUCalibrateGyroscope);
    mu_add_test(testIMUCalibrateAccelerometer);
    mu_add_test(testIMUCalibrateMagnetometer);
}

mu_run_tests(testSuite)
