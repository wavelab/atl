#include "awesomo/camera_mount.hpp"


CameraMount::CameraMount(float roll, float pitch, float yaw, float x, float y, float z)
{
    this->pose = Pose(roll, pitch, yaw, x, y, z);
    this->setGimbalLimits(0, 0, 0, 0, 0, 0); // make sure gimbal limits are initalized
}


CameraMount::CameraMount(std::map<std::string, std::string> configs)
{

    std::string config_path;
    YAML::Node config;
    YAML::Node camera_pose;
    YAML::Node gimbal_limit;

    // precheck
    if (configs.count("camera_mount") == 0){
        std::cout << "ERROR! camera_mount_config not found" << std::endl;
    }

    // load config
    config = YAML::LoadFile(configs["camera_mount"]);

    // camera pose;
    this->pose = Pose(0, 0, 0, 0, 0, 0);

    // load camera RBT (Pose) config
    camera_pose = config["camera_mount"]["RBT_between_cam_and_FC"];
    this->pose = Pose(
        deg2rad(camera_pose["roll"].as<float>()),
        deg2rad(camera_pose["pitch"].as<float>()),
        deg2rad(camera_pose["yaw"].as<float>()),
        camera_pose["x"].as<float>(),
        camera_pose["y"].as<float>(),
        camera_pose["z"].as<float>()
    );

    // load gimbal_limits
    this->setGimbalLimits(0, 0, 0, 0, 0, 0);
    gimbal_limit = config["camera_mount"]["gimbal_limits"];
    this->setGimbalLimits(
        deg2rad(gimbal_limit["roll_upper_limit"].as<float>()),
        deg2rad(gimbal_limit["roll_lower_limit"].as<float>()),
        deg2rad(gimbal_limit["pitch_upper_limit"].as<float>()),
        deg2rad(gimbal_limit["pitch_lower_limit"].as<float>()),
        deg2rad(gimbal_limit["yaw_upper_limit"].as<float>()),
        deg2rad(gimbal_limit["yaw_lower_limit"].as<float>())
    );
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

int CameraMount::setGimbalLimits(
    float roll_upper,
    float roll_lower,
    float pitch_upper,
    float pitch_lower,
    float yaw_upper,
    float yaw_lower
)
{
    this->gimbal_limits.roll_limits << roll_upper, roll_lower;
    this->gimbal_limits.pitch_limits << pitch_upper, pitch_lower;
    this->gimbal_limits.yaw_limits << yaw_upper, yaw_lower;

    return 0;
}


int CameraMount::checkLimits(float &value, Eigen::Vector2d limits)
{
    // upper limit
    if (value > limits(0)){
        value = limits(0);
    } else if (value < limits(1)){ // lower limits
        value = limits(1);
    }
    return 0;
}

int CameraMount::checkSetPointLimits(
    Eigen::Vector3d frame_rpy,
    float &roll,
    float &pitch,
    float &yaw
)
{
    Eigen::Vector2d current_roll_limits;
    Eigen::Vector2d current_pitch_limits;
    Eigen::Vector2d current_yaw_limits;

    current_roll_limits(0) = this->gimbal_limits.roll_limits(0) + frame_rpy(0);
    current_roll_limits(1) = this->gimbal_limits.roll_limits(1) + frame_rpy(0);

    current_pitch_limits(0) = this->gimbal_limits.pitch_limits(0) + frame_rpy(1);
    current_pitch_limits(1) = this->gimbal_limits.pitch_limits(1) + frame_rpy(1);

    current_yaw_limits(0) = this->gimbal_limits.yaw_limits(0) + frame_rpy(2);
    current_yaw_limits(1) = this->gimbal_limits.yaw_limits(1) + frame_rpy(2);

    this->checkLimits(roll, current_roll_limits);
    this->checkLimits(pitch, current_pitch_limits);
    this->checkLimits(yaw, current_yaw_limits);

    return 0;
}


int CameraMount::calcRollAndPitchSetpoints(
    Eigen::Vector3d target_position,
    Eigen::Quaterniond &imu,
    Eigen::Vector3d &imu_setpoint_rpy
)
{
    Eigen::Matrix3d frame_rot_mtx;
    Eigen::Vector3d frame_rpy;
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_setpoint;
    float dist;

    frame_rot_mtx = imu.toRotationMatrix();
    frame_rpy = frame_rot_mtx.eulerAngles(0, 1, 2);
    dist = target_position.norm();

    // if (abs(target_position(2)) > 0.05){ // handle case when camera is really close
    if (true){
        roll_setpoint = asin(target_position(1) / dist);
        pitch_setpoint = asin(target_position(0) / dist);

    } else {
        roll_setpoint = 0.0;
        pitch_setpoint = 0.0;
    }

    yaw_setpoint = 0.0; // unused at the moment

    std::cout << "target position \n" << target_position << std::endl;
    // this needs to be fixed. it over limits at the moment
    // this->checkSetPointLimits(frame_rpy, roll_setpoint, pitch_setpoint, yaw_setpoint);

    imu_setpoint_rpy << roll_setpoint, pitch_setpoint, yaw_setpoint;
    return 0;
}
