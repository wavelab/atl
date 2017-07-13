#ifndef ATL_ESTIMATION_KF_TRACKER_HPP
#define ATL_ESTIMATION_KF_TRACKER_HPP

#include "atl/utils/utils.hpp"

namespace atl {

#define ENBDIM "Invalid number of dimensions, have support for 1, 2, or 3!"
#define EMUSIZE "Initial mu should have size %d but actual is %d!"
#define ERROWSIZE "Motion noise R should have %d rows but got %d!"
#define ERCOLSIZE "Motion noise R should have %d columns but got %d!"
#define ECROWSIZE "Measurement C should have %d rows but got %d!"
#define ECCOLSIZE "Measurement C should have %d columns but got %d!"
#define EQROWSIZE "Measurement noise Q should have %d rows but got %d!"
#define EQCOLSIZE "Measurement noise Q should have %d columns but got %d!"
#define ECHECKCONFIG "Consider checking your dimensions in config: [%s]!"
#define EASIZE "Transition matrix A should be a square matrix of size %d!"
#define EYSIZE "Measurement vector y should be of size %d!"

#define MATRIX_A_CONSTANT_ACCELERATION_X(A) \
  A << 1.0, dt, pow(dt, 2.0) / 2.0, 0.0, 1.0, dt, 0.0, 0.0, 1.0;

#define MATRIX_A_CONSTANT_ACCELERATION_XY(A)                               \
  A << 1.0, 0.0, dt, 0.0, pow(dt, 2.0) / 2.0, 0.0, 0.0, 1.0, 0.0, dt, 0.0, \
    pow(dt, 2.0) / 2.0, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 1.0,   \
    0.0, dt, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

#define MATRIX_A_CONSTANT_ACCELERATION_XYZ(A)                                \
  A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2.0) / 2.0, 0.0, 0.0, 0.0, 1.0,  \
    0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2.0) / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0,     \
    0.0, dt, 0.0, 0.0, pow(dt, 2.0) / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, \
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0,     \
    0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,     \
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,    \
    0.0, 0.0, 0.0, 0.0, 1.0;

#define MATRIX_A_CONSTANT_VELOCITY_X(A) A << 1.0, dt, 0.0, 1.0;

#define MATRIX_A_CONSTANT_VELOCITY_XY(A)                                   \
  A << 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, \
    0.0, 1.0;

#define MATRIX_A_CONSTANT_VELOCITY_XYZ(A)                                  \
  A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, \
    1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   \
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

class KalmanFilterTracker {
public:
  bool configured;
  bool initialized;

  int mode;
  int nb_states;
  int nb_dimensions;
  double sanity_dist;
  std::string config_file;

  VecX mu;

  MatX B;
  MatX R;

  MatX C;
  MatX Q;

  MatX S;
  MatX I;
  MatX K;

  VecX mu_p;
  MatX S_p;

  KalmanFilterTracker();
  int configure(std::string config_file);
  int initialize(VecX mu);
  int checkDimensions();
  int reset(VecX mu);
  int sanityCheck(Vec3 prev_pos, Vec3 curr_pos);
  int estimate(MatX A, VecX y);
};

}  // namespace atl
#endif
