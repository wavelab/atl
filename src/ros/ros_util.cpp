#include "awesomo/ros/ros_util.hpp"

int geoMessageQuatToEigenQuat(
    const geometry_msgs::Quaternion &q,
    Eigen::Quaterniond &eigen_quat
)
{
    eigen_quat = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

int tfQuatToEigenQuat(
    tf::Quaternion q,
    Eigen::Quaterniond &eigen_quat
)
{
    eigen_quat = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}
