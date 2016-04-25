#ifndef __UTIL_H__
#define __UTIL_H__

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <Eigen/Geometry>


// FUNCTIONS
double deg2rad(double d);
double rad2deg(double d);
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
tf::Quaternion euler2quat(double roll, double pitch, double yaw);
void quat2euler(
    const geometry_msgs::Quaternion &q,
    double *roll,
    double *pitch,
    double *yaw
);

#endif
