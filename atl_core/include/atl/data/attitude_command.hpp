#ifndef ATL_DATA_ATTITUDE_COMMAND_HPP
#define ATL_DATA_ATTITUDE_COMMAND_HPP

#include "atl/utils/utils.hpp"
#include "atl/data/transform.hpp"

namespace atl {

/**
 * Attitude Command
 */
struct AttitudeCommand {
  Vec3 rpy{0.0, 0.0, 0.0};
  double throttle = 0.0;

  AttitudeCommand() {}
  AttitudeCommand(Vec4 command);
  Vec3 toEuler(const std::string &coordinate_system = "NWU");
  void print();
};

} // namespace atl
#endif
