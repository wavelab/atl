#ifndef __CAMERA_MOUNT_HPP__
#define __CAMERA_MOUNT_HPP__

# include "awesomo/util.hpp"

class CameraMount
{
public:
    Pose pose;

    CameraMount(){};
    CameraMount(
        float roll,
        float pitch,
        float yaw,
        float x,
        float y,
        float z
    );

    Eigen::Vector3d getTargetPositionBFrame(
        Eigen::Vector3d target_position
    );

    Eigen::Vector3d getTargetPositionBPFrame(
        Eigen::Vector3d target_position,
        Eigen::Quaterniond &IMU_quat
    );

};


#endif
