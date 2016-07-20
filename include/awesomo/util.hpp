#ifndef __UTIL_H__
#define __UTIL_H__

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <Eigen/Geometry>



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

    Pose(void):
        x(0),
        y(0),
        z(0),
        roll(0),
        pitch(0),
        yaw(0) {}
};

class Velocity
{
public:
    double linear_x;
    double linear_y;
    double linear_z;

    double angular_x;
    double angular_y;
    double angular_z;

    Velocity(void):
        linear_x(0),
        linear_y(0),
        linear_z(0),
        angular_x(0),
        angular_y(0),
        angular_z(0) {}
};

class Attitude
{
public:
    double x;
    double y;
    double z;
    double w;

    double roll;
    double pitch;
    double yaw;

    Attitude(void):
        x(0),
        y(0),
        z(0),
        w(0),
        roll(0),
        pitch(0),
        yaw(0) {}
};

class Position
{
public:
    double x;
    double y;
    double z;

    Position(void):
        x(0),
        y(0),
        z(0) {}
};

class LandingTargetPosition
{
public:
    bool detected;
    float x;
    float y;
    float z;

    LandingTargetPosition(void):
        detected(false),
        x(0),
        y(0),
        z(0) {}
};




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

int applyRotationToPosition(
    double roll,
    double pitch,
    double yaw,
    LandingTargetPosition &position
);

int applyRotationToPosition(
    double x,
    double y,
    double z,
    double w,
    LandingTargetPosition &position
);


tf::Quaternion euler2quat(double roll, double pitch, double yaw);
void quat2euler(
    const geometry_msgs::Quaternion &q,
    double *roll,
    double *pitch,
    double *yaw
);

#endif
