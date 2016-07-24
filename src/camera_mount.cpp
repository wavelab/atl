#include "awesomo/camera_mount.hpp"


CameraMount::CameraMount(
    float roll,
    float pitch,
    float yaw,
    float x,
    float y,
    float z
)
{
    this->pose = Pose(roll, pitch, yaw, x, y, z);
}

Eigen::Vector3d CameraMount::getTargetPositionBFrame(
    Eigen::Vector3d target_position
)
{
    return (this->pose.rotationMatrix() * target_position
            + this->pose.position);
}

Eigen::Vector3d CameraMount::getTargetPositionBPFrame(
    Eigen::Vector3d target_position,
    Eigen::Quaterniond &IMU_quat
)
{   Eigen::Quaterniond IMU_inv;
    Eigen::Matrix3d IMU_inv_rot;
    Eigen::Vector3d posInBPF;

    IMU_inv = IMU_quat.inverse();
    IMU_inv_rot = IMU_inv.toRotationMatrix();
    posInBPF = this->pose.rotationMatrix() * target_position
        + this->pose.position;
    posInBPF = IMU_inv_rot * posInBPF;

    return posInBPF;
}

