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

// TESTS
int testImu(void);

int testImu(void)
{
    IMU imu;
    int s;
    struct sockaddr_in serv_addr;

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

    // // bind to server
    // if (bind(s, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    //     perror("Failed to bind to visualizer");
    //     exit(EXIT_FAILURE);
    // }

    char buf[100];
    // float w;
    // float x;
    // float y;
    // float z;

    while (1) {
        imu.read();
        imu.print();

        sprintf(buf, "%f %f %f %f", 1.0, 1.0, 1.0, 1.0);

        sendto(
            s,
            buf,
            strlen(buf),
            0,
            (struct sockaddr *) &serv_addr,
            sizeof(serv_addr)
        );
		usleep(500000);
    }

	return 0;
}

void testSuite(void)
{
    mu_add_test(testImu);
}

mu_run_tests(testSuite)
