#include "awesomo/ros/ros_util.hpp"


int geoMessageQuatToEigenQuat(
    const geometry_msgs::Quaternion &q,
    Eigen::Quaterniond &eigen_quat
)
{
    eigen_quat.w = q.w;
    eigen_quat.x = q.x;
    eigen_quat.y = q.y;
    eigen_quat.z = q.z;
}

int tfQuatToEigenQuat(
    tf::Quaternion q,
    Eigen::Quaterniond &eigen_quat
)
{
    eigen_quat.w = q.w;
    eigen_quat.x = q.x;
    eigen_quat.y = q.y;
    eigen_quat.z = q.z;
}
