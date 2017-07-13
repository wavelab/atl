#ifndef ATL_GAZEBO_KINEMATICS_TWOWHEEL_HPP
#define ATL_GAZEBO_KINEMATICS_TWOWHEEL_HPP

#include "atl/utils/utils.hpp"

namespace atl {

/** Generic two wheel robot motion model */
class TwoWheelRobot2DModel {
public:
  /** Robot pose consisting of position in x and y (meters) and heading
   * (radians) */
  Vec3 pose;

  TwoWheelRobot2DModel() : pose{0.0, 0.0, 0.0} {}
  explicit TwoWheelRobot2DModel(const Vec3 &pose) : pose{pose} {}

  /**
   * Update two wheel model
   *
   * @param inputs Model input vector where first input is wheel velocity in
   * m/s and the second input is heading angular velocity rad/s
   *
   * @param dt Update time step in seconds
   *
   * @returns Updated pose of two wheel robot
   */
  Vec3 update(const Vec2 &inputs, double dt);
};

}  // namespace wave

#endif  // ATL_GAZEBO_KINEMATICS_TWOWHEEL_HPP
