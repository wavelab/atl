#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "munit.h"
#include "imu.hpp"
#include "util.hpp"


// CONSTANTS
#define GYRO_CONFIG_FILE "./gyroscope.yaml"
#define ACCEL_CONFIG_FILE "./accelerometer.yaml"
#define MAG_CONFIG_FILE "./magnetometer.yaml"
#define MAG_RECORD_FILE "./magnetometer.dat"


// TESTS
int testUpdate(void);
int testCalibrateGyroscope(void);
int testCalibrateAccelerometer(void);
int testCalibrateMagnetometer(void);


void transmitAccelGyroData(int s, struct sockaddr_in *server, struct IMU *imu)
{
    char filtered_data[100];
    char accel_raw[100];
    char gyro_raw[100];
    Eigen::Quaterniond q;

    // prepare filtered data string
    euler2Quaternion(
        imu->roll,
        imu->pitch,
        imu->yaw * M_PI / 180.0,
        q
    );
    memset(filtered_data, '\0', sizeof(filtered_data));
    sprintf(filtered_data, "%f %f %f %f", q.w(), q.x(), q.y(), q.z());

    // prepare aceleration data string
    euler2Quaternion(
        imu->accel->roll,
        imu->accel->pitch,
        imu->mag->bearing,
        q
    );
    memset(accel_raw, '\0', sizeof(accel_raw));
    sprintf(accel_raw, "%f %f %f %f", q.w(), q.x(), q.y(), q.z());

    // prepare gyroscope data string
    euler2Quaternion(
        imu->gyro->roll,
        imu->gyro->pitch,
        imu->mag->bearing,
        q
    );
    memset(gyro_raw, '\0', sizeof(gyro_raw));
    sprintf(gyro_raw, "%f %f %f %f", q.w(), q.x(), q.y(), q.z());

    // send filtered, accelerometer and gyroscope data
    server->sin_port = htons(7000);
    sendto(
        s,
        filtered_data,
        strlen(filtered_data),
        0,
        (struct sockaddr *) &server,
        sizeof(server)
    );

    server->sin_port = htons(7001);
    sendto(
        s,
        accel_raw,
        strlen(accel_raw),
        0,
        (struct sockaddr *) &server,
        sizeof(server)
    );

    server->sin_port = htons(7002);
    sendto(
        s,
        gyro_raw,
        strlen(gyro_raw),
        0,
        (struct sockaddr *) &server,
        sizeof(server)
    );
}

int testUpdate(void)
{
    IMU imu;
    int s;
    struct sockaddr_in server;

    // setup UDP socket
    s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("Failed to create socket");
        exit(EXIT_FAILURE);
    }

    // construct server details
    memset((char *) &server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr("192.168.1.20");
    server.sin_port = htons(7000);

    for (int i = 0; i < 100; i++) {
        imu.update();

        mu_check(fltcmp(imu.gyro->x, 0.0) != 0);
        mu_check(fltcmp(imu.gyro->y, 0.0) != 0);
        mu_check(fltcmp(imu.gyro->z, 0.0) != 0);

        mu_check(fltcmp(imu.accel->x, 0.0) != 0);
        mu_check(fltcmp(imu.accel->y, 0.0) != 0);
        mu_check(fltcmp(imu.accel->z, 0.0) != 0);

        mu_check(fltcmp(imu.mag->x, 0.0) != 0);
        mu_check(fltcmp(imu.mag->y, 0.0) != 0);
        mu_check(fltcmp(imu.mag->z, 0.0) != 0);

        transmitAccelGyroData(s, &server, &imu);
		usleep(50000);
    }

	return 0;
}

int testCalibrateGyroscope(void)
{
    IMU imu;

    imu.state = IMU_UNIT_TESTING;
    imu.calibrateGyroscope(GYRO_CONFIG_FILE);

    mu_check(fltcmp(imu.gyro->offset_x, 1.0) == 0);
    mu_check(fltcmp(imu.gyro->offset_y, 2.0) == 0);
    mu_check(fltcmp(imu.gyro->offset_z, 3.0) == 0);

    return 0;
}

int testCalibrateAccelerometer(void)
{
    IMU imu;

    imu.state = IMU_UNIT_TESTING;
    imu.calibrateAccelerometer(ACCEL_CONFIG_FILE);

    mu_check(fltcmp(imu.accel->offset_x, 2.0) == 0);
    mu_check(fltcmp(imu.accel->offset_y, 2.0) == 0);
    mu_check(fltcmp(imu.accel->offset_z, 2.0) == 0);

    return 0;
}

int testCalibrateMagnetometer(void)
{
    IMU imu;

    imu.state = IMU_UNIT_TESTING;
    imu.calibrateMagnetometer(MAG_CONFIG_FILE, MAG_RECORD_FILE);

    mu_check(fltcmp(imu.mag->offset_x, 0.5) != 0);
    mu_check(fltcmp(imu.mag->offset_y, 1.0) != 0);
    mu_check(fltcmp(imu.mag->offset_z, 1.5) != 0);

    mu_check(fltcmp(imu.mag->scale_x, 0.0) != 0);
    mu_check(fltcmp(imu.mag->scale_y, 0.0) != 0);
    mu_check(fltcmp(imu.mag->scale_z, 0.0) != 0);

    return 0;
}

void testSuite(void)
{
    // mu_add_test(testUpdate);
    mu_add_test(testCalibrateGyroscope);
    mu_add_test(testCalibrateAccelerometer);
    mu_add_test(testCalibrateMagnetometer);
}

mu_run_tests(testSuite)
