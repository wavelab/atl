#ifndef __AWESOMO_UTILS_DATA_HPP__
#define __AWESOMO_UTILS_DATA_HPP__

#include <iostream>
#include <fstream>
#include <vector>

#include "awesomo_core/utils/math.hpp"


namespace awesomo {

// CSV ERROR MESSAGES
#define E_CSV_DATA_LOAD "Error! failed to load test data [%s]!!\n"
#define E_CSV_DATA_OPEN "Error! failed to open file for output [%s]!!\n"


class AttitudeCommand {
public:
  Quaternion orientation;
  double throttle;

  AttitudeCommand(void);
  AttitudeCommand(Vec4 command);
  void print(void);
};

class Pose {
public:
  Vec3 position;
  Quaternion orientation;

  Pose(void);
  Pose(Vec3 position, Quaternion orientation);
  Pose(double roll, double pitch, double yaw, double x, double y, double z);
  Mat3 rotationMatrix(void);
  void printPosition(void);
  void printOrientation(void);
  void print(void);
};

class Velocity {
public:
  double linear_x;
  double linear_y;
  double linear_z;

  double angular_x;
  double angular_y;
  double angular_z;

  Velocity(void) {
    this->linear_x = 0;
    this->linear_y = 0;
    this->linear_z = 0;
    this->angular_x = 0;
    this->angular_y = 0;
    this->angular_z = 0;
  }
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

  Attitude(void) {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->w = 0;
    this->roll = 0;
    this->pitch = 0;
    this->yaw = 0;
  }
};

class LandingTargetPosition {
public:
  bool detected;
  Vec3 position;

  LandingTargetPosition(void) {
    this->detected = false;
    this->position << 0, 0, 0;
  }
};

int csvrows(std::string file_path);
int csvcols(std::string file_path);
int csv2mat(std::string file_path, bool header, MatX &data);
int mat2csv(std::string file_path, MatX data);

}  // end of awesomo namespace
#endif
