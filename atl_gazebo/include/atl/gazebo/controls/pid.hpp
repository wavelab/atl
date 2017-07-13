#ifndef ATL_GAZEBO_CONTROLS_PID_HPP
#define ATL_GAZEBO_CONTROLS_PID_HPP

namespace atl {

/** PID Controller */
class PID {
public:
  double error_prev;
  double error_sum;

  double error_p;
  double error_i;
  double error_d;

  double k_p;
  double k_i;
  double k_d;

  PID()
      : error_prev(0.0),
        error_sum(0.0),
        error_p(0.0),
        error_i(0.0),
        error_d(0.0),
        k_p(0.0),
        k_i(0.0),
        k_d(0.0) {}

  PID(double k_p, double k_i, double k_d)
      : error_prev(0.0),
        error_sum(0.0),
        error_p(0.0),
        error_i(0.0),
        error_d(0.0),
        k_p(k_p),
        k_i(k_i),
        k_d(k_d) {}

  double update(double setpoint, double actual, double dt);
};

}  // namespace atl
#endif  // ATL_GAZEBO_CONTROLS_PID_HPP
