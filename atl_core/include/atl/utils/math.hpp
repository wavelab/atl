#ifndef ATL_UTILS_MATH_HPP
#define ATL_UTILS_MATH_HPP

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

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

typedef Eigen::Quaterniond Quaternion;
#endif

/**
 * Create random integer
 * @param ub Upper bound
 * @param lb Lower bound
 * @return Random integer
 */
int randi(int ub, int lb);

/**
 * Create random double
 * @param ub Upper bound
 * @param lb Lower bound
 * @return Random floating point
 */
double randf(double ub, double lb);

/**
 * Sign of number
 * @param x Number to check sign
 * @return
 *    - 0: Number is zero
 *    - 1: Positive number
 *    - -1: Negative number
 */
int sign(double x);

/**
 * Floating point comparator
 * @param f1 First value
 * @param f2 Second value
 * @return
 *    - 0: if equal
 *    - 1: if f1 > f2
 *    - -1: if f1 < f2
 */
int fltcmp(double f1, double f2);

/**
 * Calculate median given an array of numbers
 * @param v Array of numbers
 * @return Median of given array
 */
double median(std::vector<double> v);

/**
 * Degrees to radians
 * @param d Degree to be converted
 * @return Degree in radians
 */
double deg2rad(double d);

/**
 * Radians to degree
 * @param r Radian to be converted
 * @return Radian in degrees
 */
double rad2deg(double r);

/**
 * Load std::vector of doubles to an Eigen::Matrix
 * @param x Matrix values
 * @param rows Number of matrix rows
 * @param cols Number of matrix colums
 * @param y Output matrix
 */
void load_matrix(std::vector<double> x, int rows, int cols, MatX &y);

/**
 * Load an Eigen::Matrix into a std::vector of doubles
 * @param A Matrix
 * @param x Output vector of matrix values
 */
void load_matrix(MatX A, std::vector<double> &x);

/**
 * Convert Euler angles to rotation matrix
 * @param euler Input Euler angles
 * @param euler_seq Euler angle sequence
 * @param R Output rotation matrix
 */
int euler2rot(Vec3 euler, int euler_seq, Mat3 &R);

/**
 * Convert Euler angles to quaternion
 * @param euler Input Euler angles
 * @param euler_seq Euler angle sequence
 * @param q Output quaternion
 */
int euler2quat(Vec3 euler, int euler_seq, Quaternion &q);

/**
 * Convert quanternion to Euler angles
 * @param q Input quaternion
 * @param euler_seq Euler angle sequence
 * @param euler Output Euler angles
 */
int quat2euler(Quaternion q, int euler_seq, Vec3 &euler);

/**
 * Convert Quaternion to rotation matrix
 * @param q Input quaternion
 * @param R Output rotation matrix
 */
int quat2rot(Quaternion q, Mat3 &R);

/**
 * Convert from ENU to NWU
 * @param enu ENU vector
 * @param nwu NWU vector
 */
void enu2nwu(Vec3 enu, Vec3 &nwu);

void cf2nwu(Vec3 cf, Vec3 &nwu);
void cf2enu(Vec3 cf, Vec3 &nwu);

/**
 * Convert from NWU to ENU
 * @param nwu NWU vector
 * @param enu ENU vector
 */
void nwu2enu(Vec3 nwu, Vec3 &enu);

/**
 * Convert from NED to ENU
 * @param ned NED vector
 * @param enu ENU vector
 */
void ned2enu(Vec3 ned, Vec3 &enu);

/**
 * Convert from NWU to NED
 * @param nwu NWU vector
 * @param ned NED vector
 */
void nwu2ned(Quaternion nwu, Quaternion &ned);
void ned2nwu(Quaternion ned, Quaternion &enu);
void enu2nwu(Quaternion enu, Quaternion &nwu);
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
double wrapTo180(double euler_angle);
double wrapTo360(double euler_angle);
double cross_track_error(Vec2 p1, Vec2 p2, Vec2 pos);
int point_left_right(Vec2 p1, Vec2 p2, Vec2 pos);
// int closest_point(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 &closest, bool
// limit=false);
double closest_point(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 &closest);
Vec2 linear_interpolation(Vec2 a, Vec2 b, double mu);

} // namespace atl
#endif
