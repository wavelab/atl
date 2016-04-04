#include "awesomo/util.hpp"

int euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw,
                  Eigen::Quaterniond &q)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    q = yawAngle * pitchAngle * rollAngle;
    q.normalize();
    return 0;
}

int euler2RotationMatrix( const double roll,
                        const double pitch,
                        const double yaw,
                        Eigen::Matrix3d &rot)
{
    Eigen::Quaterniond q_normalized;
    euler2Quaternion(roll, pitch, yaw, q_normalized);
    rot = q_normalized.toRotationMatrix();
    return 0;
}


double deg2rad(double d)
{
    return d * (M_PI / 180);
}

double rad2deg(double d)
{
    return d * (180 / M_PI);
}

tf::Quaternion euler2quat(double roll, double pitch, double yaw)
{
    return tf::createQuaternionFromRPY(roll, pitch, yaw);
}

void quat2euler(
    const geometry_msgs::Quaternion &q,
    double *roll,
    double *pitch,
    double *yaw
)
{
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(quat);
    m.getRPY(*roll, *pitch, *yaw);
}
