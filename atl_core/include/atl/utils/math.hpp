#ifndef ATL_UTILS_MATH_HPP
#define ATL_UTILS_MATH_HPP

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "atl/utils/log.hpp"

namespace atl {

#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::VectorXd VecX;

typedef Eigen::Matrix2d Mat2;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::MatrixXd MatX;

typedef Eigen::Matrix<double, 3, 4> Mat34;

typedef Eigen::Quaterniond Quaternion;
#endif

/**
 * Create random integer
 * @param ub Upper bound
 * @param lb Lower bound
 * @return Random integer
 */
int randi(const int ub, const int lb);

/**
 * Create random double
 * @param ub Upper bound
 * @param lb Lower bound
 * @return Random floating point
 */
double randf(const double ub, const double lb);

/**
 * Sign of number
 * @param x Number to check sign
 * @return
 *    - 0: Number is zero
 *    - 1: Positive number
 *    - -1: Negative number
 */
int sign(const double x);

/**
 * Floating point comparator
 * @param f1 First value
 * @param f2 Second value
 * @return
 *    - 0: if equal
 *    - 1: if f1 > f2
 *    - -1: if f1 < f2
 */
int fltcmp(const double f1, const double f2);

/**
 * Calculate median given an array of numbers
 * @param v Array of numbers
 * @return Median of given array
 */
double median(const std::vector<double> &v);

/**
 * Degrees to radians
 * @param d Degree to be converted
 * @return Degree in radians
 */
double deg2rad(const double d);

/**
 * Radians to degree
 * @param r Radian to be converted
 * @return Radian in degrees
 */
double rad2deg(const double r);

/**
 * Right hand rotation Matrix in x-axis
 * @param angle Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 rotx(const double angle);

/**
 * Right hand rotation Matrix in y-axis
 * @param angle Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 roty(const double angle);

/**
 * Right hand rotation Matrix in z-axis
 * @param angle Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 rotz(const double angle);

/**
 * Load std::vector of doubles to an Eigen::Matrix
 * @param x Matrix values
 * @param rows Number of matrix rows
 * @param cols Number of matrix colums
 * @param y Output matrix
 */
void load_matrix(const std::vector<double> &x,
                 const int rows,
                 const int cols,
                 MatX &y);

/**
 * Load an Eigen::Matrix into a std::vector of doubles
 * @param A Matrix
 * @param x Output vector of matrix values
 */
void load_matrix(const MatX A, std::vector<double> &x);

/**
 * Convert Euler angles to rotation matrix
 * @param euler Input Euler angles
 * @param euler_seq Euler angle sequence
 * @return R Output rotation matrix
 */
int euler2rot(const Vec3 &euler, const int euler_seq, Mat3 &R);

/**
 * Convert Euler angles to quaternion
 * @param euler Input Euler angles
 * @param euler_seq Euler angle sequence
 * @param q Output quaternion
 */
int euler2quat(const Vec3 &euler, const int euler_seq, Quaternion &q);

/**
 * Convert quanternion to Euler angles
 * @param q Input quaternion
 * @param euler_seq Euler angle sequence
 * @param euler Output Euler angles
 */
int quat2euler(const Quaternion &q, const int euler_seq, Vec3 &euler);

/**
 * Convert Quaternion to rotation matrix
 * @param q Input quaternion
 * @param R Output rotation matrix
 */
Mat3 quat2rot(const Quaternion &q);

/**
 * Convert from ENU to NWU
 * @param enu ENU vector
 * @return nwu NWU vector
 */
Vec3 enu2nwu(const Vec3 &enu);

/**
 * Convert from EDN to NWU
 * @param enu EDN vector
 * @return nwu NWU vector
 */
Vec3 edn2nwu(const Vec3 &edn);

/**
 * Convert from EDN to ENU
 * @param enu EDN vector
 * @return enu ENU vector
 */
Vec3 edn2enu(const Vec3 &edn);

/**
 * Convert from NWU to ENU
 * @param nwu NWU vector
 * @return enu ENU vector
 */
Vec3 nwu2enu(const Vec3 &nwu);

/**
 * Convert from NED to ENU
 * @param ned NED vector
 * @return enu ENU vector
 */
Vec3 ned2enu(const Vec3 &ned);

/**
 * Convert from NWU to NED
 * @param nwu NWU quaternion
 * @return ned NED quaternion
 */
Quaternion nwu2ned(const Quaternion &nwu);

/**
 * Conver from NED to NWU
 * @param ned NED quaternion
 * @param nwu NWU quaternion
 */
Quaternion ned2nwu(const Quaternion &ned);

/**
 * Conver from ENU to NWU
 * @param ned ENU quaternion
 * @param nwu NWU quaternion
 */
Quaternion enu2nwu(const Quaternion &enu);

void target2body(Vec3 target_pos_if,
                 Vec3 body_pos_if,
                 Quaternion body_orientation_if,
                 Vec3 &target_pos_bf);
void target2body(Vec3 target_pos_if,
                 Vec3 body_pos_if,
                 Vec3 body_euler_if,
                 Vec3 &target_pos_bf);
void target2bodyplanar(Vec3 target_pos_if,
                       Vec3 body_pos_if,
                       Quaternion body_orientation_if,
                       Vec3 &target_pos_bf);
void target2bodyplanar(Vec3 target_pos_if,
                       Vec3 body_pos_if,
                       Vec3 body_euler_if,
                       Vec3 &target_pos_bf);
void target2inertial(Vec3 target_pos_bf,
                     Vec3 body_pos_if,
                     Vec3 body_euler_if,
                     Vec3 &target_pos_if);
void target2inertial(Vec3 target_pos_bf,
                     Vec3 body_pos_if,
                     Quaternion body_orientation_if,
                     Vec3 &target_pos_if);
void inertial2body(Vec3 enu_if, Quaternion orientation_if, Vec3 &nwu_bf);
void inertial2body(Vec3 enu_if, Vec3 orientation_if, Vec3 &nwu_bf);

/**
 * Wrap angle in degrees to 180
 *
 * @param d Degrees
 * @return Angle wraped to 180
 */
double wrapTo180(const double d);

/**
 * Wrap angle in degrees to 360
 *
 * @param d Degrees
 * @return Angle wraped to 360
 */
double wrapTo360(const double d);

/**
 * Wrap angle in radians to PI
 *
 * @param r Radians
 * @return Angle wraped to PI
 */
double wrapToPi(const double r);

/**
 * Wrap angle in radians to 2 PI
 *
 * @param r Radians
 * @return Angle wraped to 2 PI
 */
double wrapTo2Pi(const double r);

double cross_track_error(const Vec2 &p1, const Vec2 &p2, const Vec2 &pos);
int point_left_right(const Vec2 &p1, const Vec2 &p2, const Vec2 &pos);
// int closest_point(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 &closest, bool
// limit=false);
double
closest_point(const Vec2 &p1, const Vec2 &p2, const Vec2 &p3, Vec2 &closest);
Vec2 linear_interpolation(const Vec2 &a, const Vec2 &b, const double mu);

} // namespace atl
#endif
