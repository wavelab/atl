#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "imu.hpp"


// CONSTANTS
#define IMU_RECORD_FILE "/tmp/imu.dat"


static void help(void)
{
    std::cout << "usage: imu_visualizer [IP address to transmit data]";
    std::cout << std::endl;
}

static void transmitData(int s, struct sockaddr_in *server, IMU &imu)
{
    char filtered_data[100];
    char accel_raw[100];
    char gyro_raw[100];
    Eigen::Quaterniond q;

    // prepare filtered data string
    euler2Quaternion(
        imu.roll,
        imu.pitch,
        imu.yaw * M_PI / 180.0,
        q
    );
    memset(filtered_data, '\0', sizeof(filtered_data));
    sprintf(filtered_data, "%f %f %f %f", q.w(), q.x(), q.y(), q.z());

    // prepare aceleration data string
    euler2Quaternion(
        imu.accel->roll,
        imu.accel->pitch,
        imu.mag->bearing,
        q
    );
    memset(accel_raw, '\0', sizeof(accel_raw));
    sprintf(accel_raw, "%f %f %f %f", q.w(), q.x(), q.y(), q.z());

    // prepare gyroscope data string
    euler2Quaternion(
        imu.gyro->roll,
        imu.gyro->pitch,
        imu.mag->bearing,
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
        (struct sockaddr *) server,
        sizeof(*server)
    );

    server->sin_port = htons(7001);
    sendto(
        s,
        accel_raw,
        strlen(accel_raw),
        0,
        (struct sockaddr *) server,
        sizeof(*server)
    );

    server->sin_port = htons(7002);
    sendto(
        s,
        gyro_raw,
        strlen(gyro_raw),
        0,
        (struct sockaddr *) server,
        sizeof(*server)
    );

    usleep(50 * 1000);
}

int main(int argc, char **argv)
{
    IMU imu;
    int s;
    struct sockaddr_in server;
    std::ofstream record_file;

    // pre-check
    if (argc < 2) {
        help();
    }

    // setup
    imu.initialize();
    record_file.open(IMU_RECORD_FILE);
    record_file << "t,us,ax,ay,az,gx,gy,gz,mx,my,mz" << std::endl;

    // setup UDP socket
    s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("Failed to create socket");
        exit(EXIT_FAILURE);
    }

    // construct server details
    memset((char *) &server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(argv[1]);
    server.sin_port = htons(7000);

    timeval time_current;
    gettimeofday(&time_current, NULL);

    // test imu update
    while (1) {
        imu.update();
        // imu.print();

        // record imu data
        // record_file << (unsigned) time(NULL) << ",";
        gettimeofday(&time_current, NULL);

        record_file << time_current.tv_sec << ",";
        record_file << time_current.tv_usec << ",";

        record_file << imu.accel->x << ",";
        record_file << imu.accel->y << ",";
        record_file << imu.accel->z << ",";

        record_file << imu.gyro->x << ",";
        record_file << imu.gyro->y << ",";
        record_file << imu.gyro->z << ",";

        record_file << imu.mag->x << ",";
        record_file << imu.mag->y << ",";
        record_file << imu.mag->z << std::endl;

        // transmit
        // transmitData(s, &server, imu);
    }

    // clean up
    record_file.close();

	return 0;
}
