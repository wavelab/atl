#ifndef ATL_CONTROL_WAYPOINT_CONTROLLER_HPP
#define ATL_CONTROL_WAYPOINT_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/utils/utils.hpp"
#include "atl/control/pid.hpp"

namespace atl {

/**
 * Waypoint Controller
 */
class WaypointController {
public:
  bool configured;

  double dt;
  PID vx_controller;
  PID vy_controller;
  PID vz_controller;

  double roll_limit[2];
  double pitch_limit[2];
  double throttle_limit[2];

  Vec3 setpoints;
  Vec4 outputs;
  AttitudeCommand att_cmd;

  WaypointController()
      : configured{false},
        dt{0.0},
        vx_controller{},
        vy_controller{},
        vz_controller{},
        roll_limit{0.0, 0.0},
        pitch_limit{0.0, 0.0},
        throttle_limit{0.0, 0.0},
        setpoints{Vec3::Zero()},
        outputs{Vec4::Zero()},
        AttitudeCommand{} {}

  int configure(const std::string &config_file);
  Vec4 calculate(const Vec3 &setpoints, const Vec3 &actual, double dt);
  void reset();
  void printOutputs();
  void printErrors();
};

}  // namespace atl
#endif
