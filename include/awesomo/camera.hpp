#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

# include "awesomo/util.hpp"



class CameraMountRBT
{
public:
    Eigen::Matrix3d camRotation;
    Eigen::Vector3d camTranslation;
    Eigen::Matrix4d camMirroring;  // apply a mirroring around an axis
    Eigen::Matrix4d camRBT;  // Rigid Body Transformation matrix
    int mirror_initialized = 0;

    CameraMountRBT(){};
    int initialize(
        double camRoll,
        double camPitch,
        double camYaw,
        double camTranslationX,
        double camTranslationY,
        double camTranslationZ
    );

    int initializeMirrorMtx(
        double camMirrorX,
        double camMirrorY,
        double camMirrorZ
    );
    int convertPoseToMtx(
        Pose &poseIn,
        Eigen::Matrix4d &poseMtxOut
    );
    int convertPositionToVector(
        LandingTargetPosition &positionIn,
        Eigen::Vector4d &positionVectorOut
    );
    int applyMirrorToPoseMtx(Eigen::Matrix4d &poseIn);
    int applyMirrorToPositionVector(Eigen::Vector4d &positionIn);
    int applyRBTtoPosition(LandingTargetPosition &positionIn);
};

#endif
