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

// TESTS
int testImu(void);


int testImu(void)
{
    IMU imu;
    int s;
    struct sockaddr_in serv_addr;
    struct sockaddr_in serv_addr2;
    struct sockaddr_in serv_addr3;

    // setup UDP socket
    s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("Failed to create socket");
        exit(EXIT_FAILURE);
    }

    // construct server details
    memset((char *) &serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr("192.168.1.20");
    serv_addr.sin_port = htons(7000);

    memset((char *) &serv_addr2, 0, sizeof(serv_addr2));
    serv_addr2.sin_family = AF_INET;
    serv_addr2.sin_addr.s_addr = inet_addr("192.168.1.20");
    serv_addr2.sin_port = htons(7001);

    memset((char *) &serv_addr3, 0, sizeof(serv_addr3));
    serv_addr3.sin_family = AF_INET;
    serv_addr3.sin_addr.s_addr = inet_addr("192.168.1.20");
    serv_addr3.sin_port = htons(7002);

    char filtered_data[100];
    char accel_raw[100];
    char gyro_raw[100];
    Eigen::Quaterniond q;

    while (1) {
        imu.update();
        printf("roll: %f pitch: %f yaw: %f\n", imu.roll, imu.pitch, imu.yaw);

        euler2Quaternion(imu.roll, imu.pitch, imu.yaw, q);
        memset(filtered_data, '\0', sizeof(filtered_data));
        sprintf(filtered_data, "%f %f %f %f", q.w(), q.x(), q.y(), q.z());

        // euler2Quaternion(imu.accel_data->roll, imu.accel_data->pitch, imu.accel_data->yaw, q);
        // memset(accel_raw, '\0', sizeof(accel_raw));
        // sprintf(accel_raw, "%f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //
        // euler2Quaternion(imu.gyro_data->roll, imu.gyro_data->pitch, imu.gyro_data->yaw, q);
        // memset(gyro_raw, '\0', sizeof(gyro_raw));
        // sprintf(gyro_raw, "%f %f %f %f", q.w(), q.x(), q.y(), q.z());

        sendto(
            s,
            filtered_data,
            strlen(filtered_data),
            0,
            (struct sockaddr *) &serv_addr,
            sizeof(serv_addr)
        );

        // sendto(
        //     s,
        //     accel_raw,
        //     strlen(accel_raw),
        //     0,
        //     (struct sockaddr *) &serv_addr2,
        //     sizeof(serv_addr2)
        // );
        //
        // sendto(
        //     s,
        //     gyro_raw,
        //     strlen(gyro_raw),
        //     0,
        //     (struct sockaddr *) &serv_addr3,
        //     sizeof(serv_addr3)
        // );

		// usleep(500000);
		usleep(50000);
    }

	return 0;
}

void testSuite(void)
{
    mu_add_test(testImu);
}

mu_run_tests(testSuite)
