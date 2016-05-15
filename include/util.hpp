#ifndef __UTIL_H__
#define __UTIL_H__

#include <cstdio>
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <Eigen/Geometry>



// CONSTANTS
#define NONBLOCK_ENABLE 0
#define NONBLOCK_DISABLE 1



// CLASSES
class Pose
{
public:
    double x;
    double y;
    double z;

    double roll;
    double pitch;
    double yaw;
};

class Orientation
{
public:
    double roll;
    double pitch;
    double yaw;
};

class Position
{
public:
    double x;
    double y;
    double z;
};



// FUNCTIONS
double deg2rad(double d);
double rad2deg(double d);
int fltcmp(float v1, float v2);
int euler2Quaternion(
    const double roll,
    const double pitch,
    const double yaw,
    Eigen::Quaterniond &q
);
int euler2RotationMatrix(
    const double roll,
    const double pitch,
    const double yaw,
    Eigen::Matrix3d &rot
);
int kbhit(void);
void nonblock(int state);

// tf::Quaternion euler2quat(double roll, double pitch, double yaw);
// void quat2euler(
//     const geometry_msgs::Quaternion &q,
//     double *roll,
//     double *pitch,
//     double *yaw
// );

#endif
