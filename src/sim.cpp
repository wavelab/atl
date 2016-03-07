#include "awesomo/sim.hpp"


double deg2rad(double degrees)
{
    return degrees * MATH_PI / 180.0;
}

void qsim_rotation_matrix(struct qsim *q, Eigen::Matrix3d &m)
{
    double phi;
    double theta;
    double psi;

    /* setup */
    phi = q->orientation(2);
    theta = q->orientation(1);
    psi = q->orientation(0);

    /* rotation matrix */
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

void qsim_inertia_matrix(struct qsim *q, double x, double y, double z)
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

void qsim_setup(struct qsim *q)
{
    /* parameters */
    q->m = 0.0f;    /* mass */
    q->L = 1.0f;    /* length of motor arm */
    q->kd = 0.0f;   /* global drag coefficient */
    q->b = 0.0f;    /* propeller drag coefficient */
    q->k = 0.0f;    /* thrust coefficient */
}

void qsim_calculate_thrust(struct qsim *q)
{
    double sum;
    Eigen::Vector3d thrust_body_frame;
    Eigen::Matrix3d rot_mat;

    /* calculate thrust in body frame */
    sum = 0;
    sum += pow(q->rotors(0), 2);
    sum += pow(q->rotors(1), 2);
    sum += pow(q->rotors(2), 2);
    sum += pow(q->rotors(3), 2);
    thrust_body_frame << 0.0f, 0.0f, sum;

    /* convert thrust to inertia frame */
    qsim_rotation_matrix(q, rot_mat);
    q->thrust = rot_mat * thrust_body_frame;
}

void qsim_calculate_drag(struct qsim *q)
{
    q->drag = -1 * q->kd * q->velocity;
}

void qsim_calculate_torque(struct qsim *q)
{
    /* torque */
    q->torque(0) = q->L * q->k * (pow(q->rotors(0), 2) - pow(q->rotors(2), 2));
    q->torque(1) = q->L * q->k * (pow(q->rotors(1), 2) - pow(q->rotors(3), 2));
    q->torque(2) = q->b * (
        pow(q->rotors(0), 2) - pow(q->rotors(1), 2)
        + pow(q->rotors(2), 2) - pow(q->rotors(3), 2)
    );
}

int qsim_calculate_acceleration(struct qsim *q, struct world *w)
{
    double mass_inv;
	double world_force_z;

    /* calculate acceleration */
    qsim_calculate_thrust(q);
    qsim_calculate_drag(q);

    std::cout << "thrust:";
    std::cout << q->thrust << std::endl;
    std::cout << std::endl;

    std::cout << "drag:";
    std::cout << q->drag << std::endl;
    std::cout << std::endl;

    mass_inv = 1 / q->m;
	world_force_z = -1 * w->gravity(2);
    q->acceleration(0) = 0 + (mass_inv * q->thrust(0)) + q->drag(0);
    q->acceleration(1) = 0 + (mass_inv * q->thrust(1)) + q->drag(1);
    q->acceleration(2) = world_force_z + (mass_inv * q->thrust(2)) + q->drag(2);

    return 0;
}

void qsim_convert_angular_velocity_to_body_frame(struct qsim *q)
{
    double phi;
    double theta;
    Eigen::Matrix3d m;

    /* setup */
    phi = q->orientation(2);
    theta = q->orientation(1);

    /* angular velocity in body frame */
    m(0) = 1;
    m(3) = 0;
    m(6) = 0;

    m(1) = 0;
    m(4) = cos(phi);
    m(7) = -1 * sin(phi);

    m(2) = -1 * sin(theta);
    m(5) = cos(theta) * sin(phi);
    m(8) = cos(theta) * cos(phi);

    /* convert angular velocity in inertia frame to body frame */
    q->angular_velocity_body_frame = m * q->angular_velocity;
}

void qsim_convert_angular_velocity_to_inertial_frame(struct qsim *q)
{
    double phi;
    double theta;
    Eigen::Matrix3d m;
    Eigen::Matrix3d w;

    /* setup */
    phi = q->orientation(2);
    theta = q->orientation(1);

    /* angular velocity in body frame */
    m(0) = 1;
    m(3) = 0;
    m(6) = 0;

    m(1) = 0;
    m(4) = cos(phi);
    m(7) = -1 * sin(phi);

    m(2) = -1 * sin(theta);
    m(5) = cos(theta) * sin(phi);
    m(8) = cos(theta) * cos(phi);

    /* convert angular velocity in body frame to inertial frame */
    q->angular_velocity = m.inverse() * q->angular_velocity_body_frame;
}

int qsim_calculate_angular_acceleration(struct qsim *q)
{
    Eigen::Matrix3d inertia_inv;
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    Eigen::Vector3d c;

    /* torque */
    qsim_calculate_torque(q);

    a = q->inertia * q->angular_velocity_body_frame;
    b = q->angular_velocity_body_frame.cross(a);
    c = q->torque - b;
    q->inertia = q->inertia.inverse();
    q->angular_acceleration_body_frame = inertia_inv * c;

	return 0;
}

void qsim_record_step(FILE *out_file, struct qsim *q)
{
    char buf[9046];

    /* setup */
    memset(buf, '\0', 9046);

    /* build string */
    /* position */
    sprintf(buf, "%f,", q->position(0));
    sprintf(strlen(buf) + buf, "%f,", q->position(1));
    sprintf(strlen(buf) + buf, "%f,", q->position(2));
    /* orientation */
    sprintf(strlen(buf) + buf, "%f,", q->orientation(0));
    sprintf(strlen(buf) + buf, "%f,", q->orientation(1));
    sprintf(strlen(buf) + buf, "%f", q->orientation(2));

    /* write to file */
    fprintf(out_file, "%s\n", buf);
}

void loop(void)
{
    int i;
    struct qsim q;
    struct world w;
    FILE *output;

    /* setup world */
    w.dt = 0.05;
    w.gravity << 0.0f, 0.0f, 10.0f;

    /* setup quadcopter */
    qsim_setup(&q);

    q.orientation << deg2rad(20.0f), deg2rad(0.0f), deg2rad(0.0f);
    q.position << 0.0f, 0.0f, 5.0f;
    q.velocity << 0.0f, 0.0f, 0.0f;
    q.acceleration << 0.0f, 0.0f, 0.0f;
    q.rotors << 2.0f, 2.1f, 2.0f, 2.0f;

    q.m = 1.0f;
    q.L = 1.0f;
    q.k = 1.0f;
    q.b = 1.0f;
    q.kd = 0.1f;
    qsim_inertia_matrix(&q, 0.1f, 0.1f, 0.1f);

    /* open output file */
    output = fopen("/tmp/sim.out", "w");

    /* loop */
    for (i = 0; i < 500; i++) {
        /* compute forces, toques and accelerations */
        qsim_calculate_acceleration(&q, &w);
        qsim_convert_angular_velocity_to_body_frame(&q);
        qsim_calculate_angular_acceleration(&q);

        /* update - using euler approximation (todo: need to do rk4 at least) */
        q.angular_velocity_body_frame(0) += w.dt * q.angular_acceleration_body_frame(0);
        q.angular_velocity_body_frame(1) += w.dt * q.angular_acceleration_body_frame(1);
        q.angular_velocity_body_frame(2) += w.dt * q.angular_acceleration_body_frame(2);
        qsim_convert_angular_velocity_to_inertial_frame(&q);

        /* qsim_convert_angular_velocity_to_inertial_frame(&q); */
        q.orientation(0) += w.dt * q.angular_velocity(0);
        q.orientation(1) += w.dt * q.angular_velocity(1);
        q.orientation(2) += w.dt * q.angular_velocity(2);

        q.velocity(0) += w.dt * q.acceleration(0);
        q.velocity(1) += w.dt * q.acceleration(1);
        q.velocity(2) += w.dt * q.acceleration(2);

        q.position(0) += w.dt * q.velocity(0);
        q.position(1) += w.dt * q.velocity(1);
        q.position(2) += w.dt * q.velocity(2);

        /* record */
        qsim_record_step(output, &q);

        /* terminator */
        if (q.position(2) < -1.0f) {
            break;
        } else if (q.position(2) > 20.0f) {
            break;
        }
    }

    /* clean up */
    fclose(output);
}
