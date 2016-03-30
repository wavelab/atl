#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__


#include <Eigen/Dense>


// STRUCTURES
struct quadrotor
{
    // kinematics info
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    Eigen::Vector3d orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d angular_acceleration;

    Eigen::Vector3d angular_velocity_body_frame;
    Eigen::Vector3d angular_acceleration_body_frame;

    Eigen::Vector3d drag;
    Eigen::Vector3d torque;
    Eigen::Matrix3d inertia;
    Eigen::Vector4d rotors;
    Eigen::Vector3d thrust;

    // parameters
    double m;    // mass
    double L;    // length of motor arm
    double kd;   // global drag coefficient
    double b;    // propeller drag coefficient
    double k;    // thrust coefficient
};


// FUNCTIONS
void quadrotor_setup(struct quadrotor *q);
void quadrotor_rotation_matrix(struct quadrotor *q, Eigen::Matrix3d &m);
void quadrotor_inertia_matrix(struct quadrotor *q, double x, double y, double z);
void quadrotor_calculate_thrust(struct quadrotor *q);
void quadrotor_calculate_drag(struct quadrotor *q);
void quadrotor_calculate_torque(struct quadrotor *q);
int quadrotor_calculate_acceleration(struct quadrotor *q, double gravity);
void quadrotor_convert_angular_velocity_to_body_frame(struct quadrotor *q);
void quadrotor_convert_angular_velocity_to_inertial_frame(struct quadrotor *q);
int quadrotor_calculate_angular_acceleration(struct quadrotor *q);
int quadrotor_update(struct quadrotor *q, double gravity, double dt);

#endif
