#ifndef __AWESOMO_UTILS_MATH_HPP__
#define __AWESOMO_UTILS_MATH_HPP__

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace awesomo {

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

int fltcmp(double f1, double f2);
double deg2rad(double d);
double rad2deg(double r);
int euler2rot(Vec3 euler, int euler_seq, Mat3 &R);
int euler2quat(Vec3 euler, int euler_seq, Quaternion &q);
int quat2euler(Quaternion q, int euler_seq, Vec3 &euler);
int quat2rot(Quaternion q, Mat3 &R);
void enu2nwu(Vec3 enu, Vec3 &nwu);
void cf2nwu(Vec3 cf, Vec3 &nwu);
void nwu2enu(Vec3 nwu, Vec3 &enu);
void cf2enu(Vec3 cf, Vec3 &nwu);
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

}  // end of awesomo namespace
#endif
