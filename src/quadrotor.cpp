#include "awesomo/quadrotor.hpp"


void quadrotor_setup(struct quadrotor *q)
{
    // parameters
    q->m = 0.0f;    // mass
    q->L = 1.0f;    // length of motor arm
    q->kd = 0.0f;   // global drag coefficient
    q->b = 0.0f;    // propeller drag coefficient
    q->k = 0.0f;    // thrust coefficient
}

void quadrotor_rotation_matrix(struct quadrotor *q, Eigen::Matrix3d &m)
{
    double phi;
    double theta;
    double psi;

    // setup
    phi = q->orientation(2);
    theta = q->orientation(1);
    psi = q->orientation(0);

    // rotation matrix - RzRyRx
    m(0) = cos(phi) * cos(theta);
    m(1) = sin(phi) * cos(theta);
    m(2) = -1 * sin(theta);

    m(3) = cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi);
    m(4) = cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi);
    m(5) = cos(theta) * sin(psi);

    m(6) = sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta);
    m(7) = cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi);
    m(8) = cos(theta) * cos(psi);
}

void quadrotor_inertia_matrix(struct quadrotor *q, double x, double y, double z)
{
    q->inertia(0) = x;
    q->inertia(1) = 0;
    q->inertia(2) = 0;

    q->inertia(3) = 0;
    q->inertia(4) = y;
    q->inertia(5) = 0;

    q->inertia(6) = 0;
    q->inertia(7) = 0;
    q->inertia(8) = z;
}

void quadrotor_calculate_thrust(struct quadrotor *q)
{
    double sum;
    Eigen::Matrix3d rot_mat;
    Eigen::Vector3d thrust_body_frame;

    // calculate thrust in body frame
    sum = 0;
    sum += pow(q->rotors(0), 2);
    sum += pow(q->rotors(1), 2);
    sum += pow(q->rotors(2), 2);
    sum += pow(q->rotors(3), 2);
    thrust_body_frame << 0.0f, 0.0f, sum;

    // convert thrust to inertia frame
    quadrotor_rotation_matrix(q, rot_mat);
    q->thrust = rot_mat * thrust_body_frame;
}

void quadrotor_calculate_drag(struct quadrotor *q)
{
    q->drag = -1 * q->kd * q->velocity;
}

void quadrotor_calculate_torque(struct quadrotor *q)
{
    // torque
    q->torque(0) = q->L * q->k * (pow(q->rotors(0), 2) - pow(q->rotors(2), 2));
    q->torque(1) = q->L * q->k * (pow(q->rotors(1), 2) - pow(q->rotors(3), 2));
    q->torque(2) = q->b * (
        pow(q->rotors(0), 2) - pow(q->rotors(1), 2)
        + pow(q->rotors(2), 2) - pow(q->rotors(3), 2)
    );
}

int quadrotor_calculate_acceleration(struct quadrotor *q, double gravity)
{
    double mass_inv;

    // calculate acceleration
    quadrotor_calculate_thrust(q);
    quadrotor_calculate_drag(q);

    mass_inv = 1 / q->m;
    q->acceleration(0) = 0 + (mass_inv * q->thrust(0)) + q->drag(0);
    q->acceleration(1) = 0 + (mass_inv * q->thrust(1)) + q->drag(1);
    q->acceleration(2) = gravity + (mass_inv * q->thrust(2)) + q->drag(2);

    return 0;
}

void quadrotor_convert_angular_velocity_to_body_frame(struct quadrotor *q)
{
    Eigen::Matrix3d m;

    // convert angular velocity in inertia frame to body frame
    quadrotor_rotation_matrix(q, m);
    q->angular_velocity_body_frame = m * q->angular_velocity;
}

void quadrotor_convert_angular_velocity_to_inertial_frame(struct quadrotor *q)
{
    Eigen::Matrix3d m;

    // convert angular velocity in body frame to inertial frame
    quadrotor_rotation_matrix(q, m);
    q->angular_velocity = m.inverse() * q->angular_velocity_body_frame;
}

int quadrotor_calculate_angular_acceleration(struct quadrotor *q)
{
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    Eigen::Vector3d c;
    Eigen::Matrix3d inertia_inv;

    // torque
    quadrotor_calculate_torque(q);

    a = q->inertia * q->angular_velocity_body_frame;
    b = q->angular_velocity_body_frame.cross(a);
    c = q->torque - b;
    q->angular_acceleration_body_frame = q->inertia.inverse() * c;

	return 0;
}

int quadrotor_update(struct quadrotor *q, double gravity, double dt)
{
    // compute forces, toques and accelerations
    quadrotor_calculate_acceleration(q, gravity);
    quadrotor_convert_angular_velocity_to_body_frame(q);
    quadrotor_calculate_angular_acceleration(q);

    // update - using euler approximation (todo: need to do rk4 at least)
    q.angular_velocity_body_frame(0) += q.angular_acceleration_body_frame(0) * dt;
    q.angular_velocity_body_frame(1) += q.angular_acceleration_body_frame(1) * dt;
    q.angular_velocity_body_frame(2) += q.angular_acceleration_body_frame(2) * dt;
    quadrotor_convert_angular_velocity_to_inertial_frame(q);

    q.orientation(0) += q.angular_velocity(0) * dt;
    q.orientation(1) += q.angular_velocity(1) * dt;
    q.orientation(2) += q.angular_velocity(2) * dt;

    q.velocity(0) += q.acceleration(0) * dt;
    q.velocity(1) += q.acceleration(1) * dt;
    q.velocity(2) += q.acceleration(2) * dt;

    q.position(0) += q.velocity(0) * dt;
    q.position(1) += q.velocity(1) * dt;
    q.position(2) += q.velocity(2) * dt;

    return 0;
}
