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

    // load camera RBT (Pose) config
    camera_pose = config["RBT_between_cam_and_FC"];
    this->pose = Pose(
        deg2rad(camera_pose["roll"].as<float>()),
        deg2rad(camera_pose["pitch"].as<float>()),
        deg2rad(camera_pose["yaw"].as<float>()),
        camera_pose["x"].as<float>(),
        camera_pose["y"].as<float>(),
        camera_pose["z"].as<float>()
    );

    // load gimbal stuff
    gimbal_dev_path = config["gimbal_dev_path"].as<std::string>();
    this->sbgc = new SBGC(gimbal_dev_path);
    this->sbgc->connect();
    this->sbgc->on();

    // load gimbal_limits
    gimbal_limit = config["gimbal_limits"];
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

int Gimbal::transformTargetPositionToBPFGimbal(
    Eigen::Vector3d target,
    Eigen::Vector3d &transformed_position
)
{
    int retval;
    Eigen::Vector3d tmp;
    Eigen::Quaterniond gimbal_imu;

    // printf("target: \t");
    // printf("x: %f\t", target(0));
    // printf("y: %f\t", target(1));
    // printf("z: %f\n", target(2));

    // get data from SimpleBGC
    retval = this->sbgc->getRealtimeData();
    if (retval != 0) {
        return -1;
    }

    // convert sbgc gimbal angle to quaternion
    euler2Quaternion(
        deg2rad(this->sbgc->data.camera_angles(0)),
        deg2rad(this->sbgc->data.camera_angles(1)),
        0.0,
        gimbal_imu
    );

    // transform tag in camera frame to quadrotor frame to global frame
    // tmp = gimbal_imu.toRotationMatrix() * this->getTargetPositionBFrame(target);
    tmp = gimbal_imu.toRotationMatrix() * this->pose.rotationMatrix() * target + this->pose.position;
    transformed_position(0) = tmp(0);
    transformed_position(1) = tmp(1);
    transformed_position(2) = tmp(2);

    return 0;
}

int Gimbal::transformTargetPositionToBPFGimbal2(
    Eigen::Vector3d target,
    Eigen::Quaterniond frame_imu,
    Eigen::Vector3d &transformed_position
)
{
    int retval;
    Eigen::Vector3d tmp;
    Eigen::Quaterniond gimbal_imu;

    // get data from SimpleBGC
    retval = this->sbgc->getRealtimeData();
    if (retval != 0) {
        return -1;
    }

    // convert sbgc gimbal angle to quaternion
    euler2Quaternion(
        deg2rad(this->sbgc->data.camera_angles(0)),
        deg2rad(this->sbgc->data.camera_angles(1)),
        0.0,
        gimbal_imu
    );

    // transform tag in camera frame to quadrotor frame to global frame
    tmp = gimbal_imu.toRotationMatrix() * this->getTargetPositionBPFrame(target, frame_imu);
    transformed_position(0) = tmp(0);
    transformed_position(1) = tmp(1);
    transformed_position(2) = tmp(2);

    return 0;
}

int Gimbal::setGimbalLimits(
    float roll_min,
    float roll_max,
    float pitch_min,
    float pitch_max,
    float yaw_min,
    float yaw_max
)
{
    this->gimbal_limits.roll_limits << roll_min, roll_max;
    this->gimbal_limits.pitch_limits << pitch_min, pitch_max;
    this->gimbal_limits.yaw_limits << yaw_min, yaw_max;

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

int Gimbal::trackTarget(Eigen::Vector3d target, Eigen::Quaterniond &imu)
{
    Eigen::Matrix3d frame_rot_mtx;
    Eigen::Vector3d frame_rpy;
    double roll_setpoint;
    double pitch_setpoint;
    double yaw_setpoint;
    double dist;

    frame_rot_mtx = imu.toRotationMatrix();
    frame_rpy = frame_rot_mtx.eulerAngles(0, 1, 2);

    dist = target.norm();
    printf("dist: %f\n", dist);

    // calculate roll pitch yaw setpoints
    // dist = sqrt(pow(target(1), 2) + pow(target(2), 2));
    roll_setpoint = asin(target(1) / dist);

    // dist = sqrt(pow(target(0), 2) + pow(target(2), 2));
    pitch_setpoint = asin(target(0) / dist);

    yaw_setpoint = 0.0; // unused at the moment

    // this needs to be fixed. it over limits at the moment
    // this->checkSetPointLimits(frame_rpy, roll_setpoint, pitch_setpoint, yaw_setpoint);

    // convert setpoints to degrees
    roll_setpoint = rad2deg(-1.0 * roll_setpoint);
    pitch_setpoint = rad2deg(1.0 * pitch_setpoint);

    // printf("roll setpoint: %f\t", roll_setpoint);
    // printf("pitch_setpoint: %f\t", pitch_setpoint);
    printf("target: \t");
    printf("x: %f\t", target(0));
    printf("y: %f\t", target(1));
    printf("z: %f\n", target(2));

    // set angle
    // this->sbgc->setAngle(roll_setpoint, pitch_setpoint, 0);
    this->sbgc->setAngle(roll_setpoint, 0, 0);
    // this->sbgc->setAngle(0, 0, 0);

    return 0;
}

int Gimbal::setGimbalAngles(double roll, double pitch, double yaw)
{
    return this->sbgc->setAngle(roll, pitch, yaw);
}
