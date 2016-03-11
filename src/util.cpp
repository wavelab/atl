#include "awesomo/util.hpp"


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
