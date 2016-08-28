#include "awesomo/gimbal.hpp"


Gimbal::Gimbal(float roll, float pitch, float yaw, float x, float y, float z)
{
    this->pose = Pose(roll, pitch, yaw, x, y, z);
    this->setGimbalLimits(0, 0, 0, 0, 0, 0); // make sure gimbal limits are initalized
}

Gimbal::Gimbal(std::map<std::string, std::string> configs)
{

    std::string config_path;
    std::string gimbal_dev_path;
    YAML::Node config;
    YAML::Node camera_pose;
    YAML::Node gimbal_limit;

    // precheck
    if (configs.count("gimbal") == 0){
        std::cout << "ERROR! gimbal_config not found" << std::endl;
    }

    // load config
    config = YAML::LoadFile(configs["gimbal"]);

    // camera pose;
    this->pose = Pose(0, 0, 0, 0, 0, 0);

    // load camera RBT (Pose) config
    camera_pose = config["gimbal"]["RBT_between_cam_and_FC"];
    this->pose = Pose(
        deg2rad(camera_pose["roll"].as<float>()),
        deg2rad(camera_pose["pitch"].as<float>()),
        deg2rad(camera_pose["yaw"].as<float>()),
        camera_pose["x"].as<float>(),
        camera_pose["y"].as<float>(),
        camera_pose["z"].as<float>()
    );

    // load gimbal stuff
    gimbal_dev_path = config["gimbal"]["gimbal_dev_path"].as<std::string>();
    this->sbgc = new SBGC(gimbal_dev_path, 115200, 500);
    this->sbgc->connect();
    this->sbgc->on();

    // load gimbal_limits
    this->setGimbalLimits(0, 0, 0, 0, 0, 0);
    gimbal_limit = config["gimbal"]["gimbal_limits"];
    this->setGimbalLimits(
        deg2rad(gimbal_limit["roll_upper_limit"].as<float>()),
        deg2rad(gimbal_limit["roll_lower_limit"].as<float>()),
        deg2rad(gimbal_limit["pitch_upper_limit"].as<float>()),
        deg2rad(gimbal_limit["pitch_lower_limit"].as<float>()),
        deg2rad(gimbal_limit["yaw_upper_limit"].as<float>()),
        deg2rad(gimbal_limit["yaw_lower_limit"].as<float>())
    );
}

Eigen::Vector3d Gimbal::getTargetPositionBFrame(Eigen::Vector3d target)
{
    return (this->pose.rotationMatrix() * target + this->pose.position);
}

Eigen::Vector3d Gimbal::getTargetPositionBPFrame(
    Eigen::Vector3d target,
    Eigen::Quaterniond &imu
)
{
    return imu.toRotationMatrix() * this->getTargetPositionBFrame(target);
}

Eigen::Vector3d Gimbal::getTargetPositionBPFGimbal(
    Eigen::Vector3d target
)
{
    int retval;
    Eigen::Quaterniond gimbal_imu;

    retval = this->sbgc->getRealtimeData();
    if (retval == 0) {
        this->sbgc->data.printData();
    }

    euler2Quaternion(
        this->sbgc->data.rc_angles(0),
        this->sbgc->data.rc_angles(1),
        this->sbgc->data.rc_angles(2),
        gimbal_imu
    );

    return gimbal_imu.toRotationMatrix() * this->getTargetPositionBFrame(target);
}

int Gimbal::setGimbalLimits(
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

int Gimbal::checkLimits(float &value, Eigen::Vector2d limits)
{
    // upper limit
    if (value > limits(0)){
        value = limits(0);
    } else if (value < limits(1)){ // lower limits
        value = limits(1);
    }
    return 0;
}

int Gimbal::checkSetPointLimits(
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

int Gimbal::calcRollAndPitchSetpoints(
    Eigen::Vector3d target_position,
    Eigen::Quaterniond &imu
)
{
    Eigen::Matrix3d frame_rot_mtx;
    Eigen::Vector3d frame_rpy;
    double roll_setpoint;
    double pitch_setpoint;
    double yaw_setpoint;
    double dist;

    frame_rot_mtx = imu.toRotationMatrix();
    frame_rpy = frame_rot_mtx.eulerAngles(0, 1, 2);
    dist = target_position.norm();

    // if (abs(target_position(2)) > 0.05){ // handle case when camera is really close
    if (true) {
        roll_setpoint = asin(target_position(1) / dist);
        pitch_setpoint = asin(target_position(0) / dist);

    } else {
        roll_setpoint = 0.0;
        pitch_setpoint = 0.0;
    }

    yaw_setpoint = 0.0; // unused at the moment

    // std::cout << "target position \n" << target_position << std::endl;
    // this needs to be fixed. it over limits at the moment
    // this->checkSetPointLimits(frame_rpy, roll_setpoint, pitch_setpoint, yaw_setpoint);
    this->sbgc->setAngle(roll_setpoint, pitch_setpoint, yaw_setpoint);

    return 0;
}

int Gimbal::setGimbalAngles(double roll, double pitch, double yaw)
{
    this->sbgc->setAngle(roll, pitch, yaw);
};



