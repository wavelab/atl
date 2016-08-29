#include "awesomo/util.hpp"

Pose::Pose(void)
{
    Eigen::Quaterniond q;

    this->q = q.setIdentity();
    this->position = Eigen::Vector3d::Zero(3,1);
}


Pose::Pose(
    float roll,
    float pitch,
    float yaw,
    float x,
    float y,
    float z
)
{
    euler2Quaternion(roll, pitch, yaw, this->q);
    this->position << x, y, z;
}

Pose::Pose(Eigen::Quaterniond q, Eigen::Vector3d position)
{
    this->q = q;
    this->position = position;
}

Eigen::Matrix3d Pose::rotationMatrix(void)
{
    return this->q.toRotationMatrix();
}


double deg2rad(double d)
{
    return d * (M_PI / 180);
}

double rad2deg(double d)
{
    return d * (180 / M_PI);
}

int fltcmp(double f1, double f2)
{
    if (fabs(f1 - f2) <= 0.0001) {
        return 0;
    } else if (f1 > f2) {
        return 1;
    } else {
        return -1;
    }
}

int euler2Quaternion(
    const double roll,
    const double pitch,
    const double yaw,
    Eigen::Quaterniond &q
)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    q = yawAngle * pitchAngle * rollAngle;
    q.normalize();

    return 0;
}

int euler2RotationMatrix(
    const double roll,
    const double pitch,
    const double yaw,
    Eigen::Matrix3d &rot
)
{
    Eigen::Quaterniond q_normalized;
    euler2Quaternion(roll, pitch, yaw, q_normalized);
    rot = q_normalized.toRotationMatrix();
    return 0;
}

inline static double sqr(double x)
{
    return x * x;
}

int linreg(std::vector<Eigen::Vector2d> pts, double *m, double *c, double *r)
{
    // linear regression of form: y = mx + c
    Eigen::Vector2d p;
    double sumx = 0.0;   /* sum of x */
    double sumx2 = 0.0;  /* sum of x^2 */
    double sumxy = 0.0;  /* sum of x * y */
    double sumy = 0.0;   /* sum of y */
    double sumy2 = 0.0;  /* sum of y^2 */

    for (int i = 0; i < pts.size(); i++) {
        p = pts[i];
        sumx += p(0);
        sumx2 += sqr(p(0));
        sumxy += p(0) * p(1);
        sumy += p(1);
        sumy2 += sqr(p(1));
    }

    double denom = (pts.size() * sumx2 - sqr(sumx));
    if (denom == 0) {
        // singular matrix. can't solve the problem.
        *m = 0;
        *c = 0;
        if (r) {
            *r = 0;
        }
        return -1;
    }

    *m = (pts.size() * sumxy  -  sumx * sumy) / denom;
    *c = (sumy * sumx2  -  sumx * sumxy) / denom;
    /* compute correlation coeff */
    if (r != NULL) {
        *r = (sumxy - sumx * sumy / pts.size());
        *r /= sqrt(
            (sumx2 - sqr(sumx) / pts.size())
            * (sumy2 - sqr(sumy) / pts.size())
        );
    }

    return 0;
}

void tic(struct timespec *tic)
{
    clock_gettime(CLOCK_MONOTONIC, tic);
}

float toc(struct timespec *tic)
{
    struct timespec toc;
    float time_elasped;

    clock_gettime(CLOCK_MONOTONIC, &toc);
    time_elasped = (toc.tv_sec - tic->tv_sec);
    time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

    return time_elasped;
}

float mtoc(struct timespec *tic)
{
    struct timespec toc;
    float time_elasped;

    clock_gettime(CLOCK_MONOTONIC, &toc);
    time_elasped = (toc.tv_sec - tic->tv_sec);
    time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000.0;

    return time_elasped;
}
