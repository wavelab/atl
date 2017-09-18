#ifndef ATL_DATA_POSITION_HPP
#define ATL_DATA_POSITION_HPP

#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Position information
 */
struct Position {
  std::string frame;
  Vec3 data{0.0, 0.0, 0.0};

  Position(const std::string &frame, const Vec3 &v) : frame{frame}, data{v} {}

  Position(const std::string &frame,
           const double x,
           const double y,
           const double z)
      : frame{frame}, data{x, y, z} {}

  /**
   * Return Homogeneous vector
   */
  Vec4 homogeneous() const {
    return Vec4{this->data(0), this->data(1), this->data(2), 1.0};
  }

  /**
   * Print position
   */
  friend std::ostream &operator<<(std::ostream &os, const Position &pos) {
    os << pos.data.transpose() << std::endl;
    return os;
  }
};

} // namespace atl
#endif
