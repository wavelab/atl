#include "awesomo/camera.hpp"


int CameraMount::initialize(
    double camRoll,
    double camPitch,
    double camYaw,
    double camTranslationX,
    double camTranslationY,
    double camTranslationZ
)
{
    // calc the rotation matrix
    euler2RotationMatrix(
        camRoll,
        camPitch,
        camYaw,
        this->camRotation
    );

    this->camTranslation << camTranslationX,
                            camTranslationY,
                            camTranslationZ;

    return 0;
}

int getAtimTargetPositionBodyFrame(
    Position target_position,
    Position &target_positionBodyFrame
)
{
    Eigen::Vector3d target_vec;
    Eigen::Vector3d target_BF_vec;

    target_vec << target_position.x
               << target_position.y
               << target_position.z;

    target_BF_vec = this->camRotation * target_vec + camTranslation;

    target_positionBodyFrame.x = target_BF_vec(0);
    target_positionBodyFrame.y = target_BF_vec(1);
    target_positionBodyFrame.z = target_BF_vec(2);

    return 0;
}


int getAtimTargetPositionBPF(
    Position target_position,
    Eigen::Quaterniond IMU_quat,
    Position &target_position_BPF
)
{
    Eigen::Vector3d target_vec;
    Eigen::Vector3d target_BPF_vec;
    Eigen::Quaterniond imu_inverse;

    target_vec << target_position.x
               << target_position.y
               << target_position.z;
    imu_inverse = IMU_quat.inverse();

    target_BFP_vec = this->camRotation * target_vec + camTranslation;
    target_BPF_vec = imu_inverse.toRotationMatrix() * target_BPF_vec;

    return 0;
}



