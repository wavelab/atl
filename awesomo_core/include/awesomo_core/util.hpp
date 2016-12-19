#ifndef __UTIL_H__
#define __UTIL_H__

#include <sys/time.h>
#include <Eigen/Geometry>


// CLASSES
class Pose {
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d position;

  Pose(void);
  Pose(float roll, float pitch, float yaw, float x, float y, float z);
  Pose(Eigen::Quaterniond q, Eigen::Vector3d position);

  Eigen::Matrix3d rotationMatrix(void);
};

class Velocity {
public:
  double linear_x;
  double linear_y;
  double linear_z;

  double angular_x;
  double angular_y;
  double angular_z;

  Velocity(void)
      : linear_x(0),
        linear_y(0),
        linear_z(0),
        angular_x(0),
        angular_y(0),
        angular_z(0) {}
};

class Attitude {
public:
  double x;
  double y;
  double z;
  double w;

  double roll;
  double pitch;
  double yaw;

  Attitude(void) : x(0), y(0), z(0), w(0), roll(0), pitch(0), yaw(0) {}
};


class LandingTargetPosition {
public:
  bool detected;
  Eigen::Vector3d position;

  LandingTargetPosition(void) : detected(false), position(0, 0, 0) {}
};


// FUNCTIONS
double deg2rad(double d);
double rad2deg(double d);
int euler2Quaternion(const double roll,
                     const double pitch,
                     const double yaw,
                     Eigen::Quaterniond &q);
int euler2RotationMatrix(const double roll,
                         const double pitch,
                         const double yaw,
                         Eigen::Matrix3d &rot);
int linreg(std::vector<Eigen::Vector2d> pts, double *m, double *b, double *r);
void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);

#endif
