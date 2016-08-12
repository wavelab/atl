#ifndef __CAMERA_MOUNT_HPP__
#define __CAMERA_MOUNT_HPP__

# include <iostream>
# include <math.h>
# include <map>
# include <yaml-cpp/yaml.h>
# include "awesomo/util.hpp"
struct GimbalLimit
{
    Eigen::Vector2d roll_limits;
    Eigen::Vector2d pitch_limits;
    Eigen::Vector2d yaw_limits;
};

class CameraMount
{
public:
    Pose pose;
    struct GimbalLimit gimbal_limits;

    CameraMount(){};
    CameraMount(float roll, float pitch, float yaw, float x, float y, float z);
    CameraMount(std::map<std::string, std::string> configs);

    Eigen::Vector3d getTargetPositionBFrame(Eigen::Vector3d target_position);
    Eigen::Vector3d getTargetPositionBPFrame(
        Eigen::Vector3d target_position,
        Eigen::Quaterniond &imu
    );

    int setGimbalLimits(
        float roll_upper,
        float roll_lower,
        float pitch_upper,
        float pitch_lower,
        float yaw_upper,
        float yaw_lower
    );

    int checkLimits(float &value, Eigen::Vector2d limits);
    int checkSetPointLimits(
        Eigen::Vector3d frame_rpy,
        float &roll,
        float &pitch,
        float &yaw
    );

    int calcRollAndPitchSetpoints(
        Eigen::Vector3d target_position,
        Eigen::Quaterniond &imu,
        Eigen::Vector3d &imu_setpoint_rpy
    );
};


#endif
