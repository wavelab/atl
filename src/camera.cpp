#include "awesomo/camera.hpp"


int CameraMountConfig::initialize(
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

    // create the rigid body transformation (RBT) matrix
    this->camRBT = Eigen::Matrix4d::Zero(4, 4);
    this->camRBT.topLeftCorner(3, 3) =  this->camRotation;
    this->camRBT.topRightCorner(3, 1) = this->camTranslation;
    this->camRBT(3, 3) = 1;


    return 0;
}

int CameraMountConfig::initializeMirrorMtx(
    double camMirrorX,
    double camMirrorY,
    double camMirrorZ
)
{
    // create mirroring matrix
    this->camMirroring = Eigen::MatrixXd::Identity(4, 4);
    this->camMirroring(0, 0) = camMirrorX;
    this->camMirroring(1, 1) = camMirrorY;
    this->camMirroring(2, 2) = camMirrorZ;

    this->mirror_initialized = 1;

    return 0;
}

int CameraMountConfig::convertPoseToMtx(
    Pose &poseIn,
    Eigen::Matrix4d &poseMtxOut
)
{
    Eigen::Matrix3d poseRotationMtx;
    euler2RotationMatrix(
        poseIn.roll,
        poseIn.pitch,
        poseIn.yaw,
        poseRotationMtx
    );

    poseMtxOut = Eigen::Matrix4d::Identity(4, 4);

    poseMtxOut.topLeftCorner(3, 3) = poseRotationMtx;
    poseMtxOut(3, 0) = poseIn.x;
    poseMtxOut(3, 1) = poseIn.y;
    poseMtxOut(3, 2) = poseIn.z;

    return 0;
}

int CameraMountConfig::convertPositionToVector(
    Position &positionIn,
    Eigen::Vector4d &positionVectorOut
)
{
    positionVectorOut(0) = positionIn.x;
    positionVectorOut(1) = positionIn.y;
    positionVectorOut(2) = positionIn.z;
    positionVectorOut(3) = 1;

    return 0;
}

int CameraMountConfig::applyMirrorToPoseMtx(Eigen::Matrix4d &poseIn)
{
    poseIn = this->camMirroring * poseIn;
    return 0;
}

int CameraMountConfig::applyMirrorToPositionVector(Eigen::Vector4d &positionIn)
{
    positionIn = this->camMirroring * positionIn;
    return 0;
}

int CameraMountConfig::applyRBTtoPosition(Position &positionIn)
{
    Eigen::Vector4d positionVect;

    this->convertPositionToVector(positionIn, positionVect);
    positionVect = this->camRBT * positionVect;

    if (this->mirror_initialized == 1) {
        positionVect = this->camMirroring * positionVect;
    }

    positionIn.x = positionVect(0);
    positionIn.y = positionVect(1);
    positionIn.z = positionVect(2);

    return 0;
}
