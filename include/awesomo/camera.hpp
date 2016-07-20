#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

# include "awesomo/util.hpp"

class CameraMount
{
public:
    Eigen::Matrix3d camRotation;
    Eigen::Vector3d camTranslation;

    CameraMount(){};
    int initialize(
        double camRoll,
        double camPitch,
        double camYaw,
        double camTranslationX,
        double camTranslationY,
        double camTranslationZ
    );


    int getAtimTargetPositionBodyFrame(
        Position target_position,
        Position &target_positionBodyFrame
    );

    int getAtimTargetPositionBPF(
        Position target_position,
        Eigen::Quaterniond IMU_quat,
        Position &target_positionBPF
    );
};


#endif
