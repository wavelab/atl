#ifndef ATL_DATA_TRANSFORM_HPP
#define ATL_DATA_TRANSFORM_HPP

#include "atl/utils/utils.hpp"

namespace atl {

class Transform : public Mat4 {
public:
  Transform() : Mat4() {}
  Transform(Mat4 mat) : Mat4(mat) {}
};

/**
 * Transform position or velocity vector of size 3
 */
Vec3 operator*(const Transform &T, const Vec3 &x);

/**
 * Transform quaternion
 */
Quaternion operator*(const Transform &T, const Quaternion &x);

/**
 * Transform from NED to NWU
 */
// clang-format off
const Transform T_nwu_ned{(Mat4() << 1.0, 0.0, 0.0, 0.0,
                                     0.0, -1.0, 0.0, 0.0,
                                     0.0, 0.0, -1.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0).finished()};
// clang-format on

/**
 * Transform from NWU to NED
 */
// clang-format off
const Transform T_ned_nwu{(Mat4() << 1.0, 0.0, 0.0, 0.0,
                                     0.0, -1.0, 0.0, 0.0,
                                     0.0, 0.0, -1.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0).finished()};
// clang-format on

/**
 * Transform from EDN to NWU
 */
// clang-format off
const Transform T_nwu_edn{(Mat4() << 0.0, 0.0, 1.0, 0.0,
                                     -1.0, 0.0, 0.0, 0.0,
                                     0.0, -1.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0).finished()};
// clang-format on

/**
 * Transform from NWU to EDN
 */
// clang-format off
const Transform T_edn_nwu{(Mat4() << 0.0, -1.0, 0.0, 0.0,
                                     0.0, 0.0, -1.0, 0.0,
                                     -1.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0).finished()};
// clang-format on

} // namespace atl
#endif
