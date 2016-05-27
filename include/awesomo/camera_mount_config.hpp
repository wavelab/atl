# include "awesomo/util.hpp"

class cameraMountConfig
{
    public:
        Eigen::Matrix3d camRotation;
        Eigen::Vector3d camTranslation;
        Eigen::Matrix4d camMirroring; //apply a mirroring around an axis
        Eigen::Matrix4d camRBT; //Rigid Body Transformation matrix
        int mirror_initialized = 0;

        cameraMountConfig(){};
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

    public:

        int convertPoseToMtx(
            Pose &poseIn,
            Eigen::Matrix4d &poseMtxOut
        );

        int convertPositionToVector(
            Position &positionIn,
            Eigen::Vector4d &positionVectorOut
        );

        int applyMirrorToPoseMtx(
            Eigen::Matrix4d &poseIn
        );

        int applyMirrorToPositionVector(
            Eigen::Vector4d &positionIn
        );


    public:

        int applyRBTtoPose(Pose &poseIn);
        int applyRBTtoPosition(Position &positionIn);
};