#include "awesomo/camera_mount.hpp"


CameraMount::CameraMount(float roll, float pitch, float yaw, float x, float y, float z)
{
    this->pose = Pose(roll, pitch, yaw, x, y, z);
}

Eigen::Vector3d CameraMount::getTargetPositionBFrame(Eigen::Vector3d target)
{
    return (this->pose.rotationMatrix() * target + this->pose.position);
}

Eigen::Vector3d CameraMount::getTargetPositionBPFrame(
    Eigen::Vector3d target,
    Eigen::Quaterniond &imu
)
{
    return imu.toRotationMatrix() * this->getTargetPositionBFrame(target);
}
