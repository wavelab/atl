#ifndef __SIM_H__
#define __SIM_H__

#include <iostream>

#include <Eigen/Dense>

#define MATH_PI 3.14159265358979323846


/* STRUCTURES */
struct world
{
    double dt;
    Eigen::Vector3d gravity;
};

struct qsim
{
    /* kinematics info */
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

    /* parameters */
    double m;    /* mass */
    double L;    /* length of motor arm */
    double kd;   /* global drag coefficient */
    double b;    /* propeller drag coefficient */
    double k;    /* thrust coefficient */
};


/* FUNCTIONS */
double deg2rad(double degrees);
void qsim_rotation_matrix(struct qsim *q, Eigen::Matrix3d &m);
void qsim_inertia_matrix(struct qsim *q, double x, double y, double z);
void qsim_setup(struct qsim *q);
void qsim_calculate_thrust(struct qsim *q);
void qsim_calculate_drag(struct qsim *q);
void qsim_calculate_torque(struct qsim *q);
int qsim_calculate_acceleration(struct qsim *q, struct world *w);
void qsim_convert_angular_velocity_to_body_frame(struct qsim *q);
void qsim_convert_angular_velocity_to_inertial_frame(struct qsim *q);
int qsim_calculate_angular_acceleration(struct qsim *q);
void qsim_record_step(FILE *out_file, struct qsim *q);
void loop(void);

#endif
