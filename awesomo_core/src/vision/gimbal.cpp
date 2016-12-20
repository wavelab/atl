#include "awesomo_core/vision/gimbal.hpp"


namespace awesomo {

Gimbal::Gimbal(
  float roll, float pitch, float yaw, float x, float y, float z) {
  this->pose = Pose(roll, pitch, yaw, x, y, z);
  this->setGimbalLimits(
    0, 0, 0, 0, 0, 0);  // make sure gimbal limits are initalized
}

Gimbal::Gimbal(std::string config_path) {
  std::string gimbal_dev_path;
  YAML::Node config;
  YAML::Node camera_pose;
  YAML::Node gimbal_limit;

  // load config
  config = YAML::LoadFile(config_path);

  // load camera RBT (Pose) config
  camera_pose = config["mount_offset"];
  this->pose = Pose(deg2rad(camera_pose["roll"].as<float>()),
                    deg2rad(camera_pose["pitch"].as<float>()),
                    deg2rad(camera_pose["yaw"].as<float>()),
                    camera_pose["x"].as<float>(),
                    camera_pose["y"].as<float>(),
                    camera_pose["z"].as<float>());

  // load gimbal stuff
  gimbal_dev_path = config["gimbal_dev_path"].as<std::string>();
  this->sbgc = new SBGC(gimbal_dev_path);
  this->sbgc->connect();
  this->sbgc->on();

  // load gimbal_limits
  gimbal_limit = config["gimbal_limits"];
  this->setGimbalLimits(deg2rad(gimbal_limit["roll_min"].as<float>()),
                        deg2rad(gimbal_limit["roll_max"].as<float>()),
                        deg2rad(gimbal_limit["pitch_min"].as<float>()),
                        deg2rad(gimbal_limit["pitch_max"].as<float>()),
                        deg2rad(gimbal_limit["yaw_min"].as<float>()),
                        deg2rad(gimbal_limit["yaw_max"].as<float>()));
}

Eigen::Vector3d Gimbal::getTargetPositionBFrame(Eigen::Vector3d target) {
  return (this->pose.rotationMatrix() * target + this->pose.position);
}

Eigen::Vector3d Gimbal::getTargetPositionBPFrame(Eigen::Vector3d target,
                                                 Eigen::Quaterniond &imu) {
  return imu.toRotationMatrix() * this->getTargetPositionBFrame(target);
}

int Gimbal::transformTargetPosition(Eigen::Vector3d target,
                                    Eigen::Vector3d &transformed_target) {
  int retval;
  Eigen::Vector3d tmp;
  Eigen::Quaterniond gimbal_imu;

  // get data from SimpleBGC
  retval = this->sbgc->getRealtimeData();
  if (retval != 0) {
    return -1;
  }

  // convert sbgc gimbal angle to quaternion
  // (making gimbal angles NED, notice the -ve sign in pitch)
  euler2Quaternion(deg2rad(this->sbgc->data.camera_angles(0)),
                   deg2rad(-this->sbgc->data.camera_angles(1)),
                   0.0,
                   gimbal_imu);

  // camera frame to camera mount frame
  tmp = this->pose.rotationMatrix().inverse() * target;
  // inverse because we want tag relative to quad
  // without it, results are relative to tag

  // camera mount frame to body planar frame
  tmp = gimbal_imu.toRotationMatrix() * tmp;
  transformed_target(0) = tmp(0);
  transformed_target(1) = tmp(1);
  transformed_target(2) = tmp(2);

  return 0;
}

int Gimbal::setGimbalLimits(float roll_min,
                            float roll_max,
                            float pitch_min,
                            float pitch_max,
                            float yaw_min,
                            float yaw_max) {
  this->gimbal_limits.roll_limits << roll_min, roll_max;
  this->gimbal_limits.pitch_limits << pitch_min, pitch_max;
  this->gimbal_limits.yaw_limits << yaw_min, yaw_max;

  return 0;
}

int Gimbal::checkLimits(float &value, Eigen::Vector2d limits) {
  // upper limit
  if (value > limits(0)) {
    value = limits(0);

    // lower limit
  } else if (value < limits(1)) {
    value = limits(1);
  }

  return 0;
}

int Gimbal::checkSetPointLimits(Eigen::Vector3d frame_rpy,
                                float roll,
                                float pitch,
                                float yaw) {
  Eigen::Vector2d roll_limits;
  Eigen::Vector2d pitch_limits;
  Eigen::Vector2d yaw_limits;

  roll_limits(0) = this->gimbal_limits.roll_limits(0) + frame_rpy(0);
  roll_limits(1) = this->gimbal_limits.roll_limits(1) + frame_rpy(0);

  pitch_limits(0) = this->gimbal_limits.pitch_limits(0) + frame_rpy(1);
  pitch_limits(1) = this->gimbal_limits.pitch_limits(1) + frame_rpy(1);

  yaw_limits(0) = this->gimbal_limits.yaw_limits(0) + frame_rpy(2);
  yaw_limits(1) = this->gimbal_limits.yaw_limits(1) + frame_rpy(2);

  this->checkLimits(roll, roll_limits);
  this->checkLimits(pitch, pitch_limits);
  this->checkLimits(yaw, yaw_limits);

  return 0;
}

int Gimbal::trackTarget(Eigen::Vector3d target) {
  double roll_setpoint;
  double pitch_setpoint;
  double yaw_setpoint;
  double dist;

  // calculate roll pitch yaw setpoints
  dist = target.norm();
  roll_setpoint = asin(target(1) / dist);
  pitch_setpoint = asin(target(0) / dist);
  yaw_setpoint = 0.0;  // unused at the moment

  // this needs to be fixed. it over limits at the moment
  // this->checkSetPointLimits(frame_rpy, roll_setpoint, pitch_setpoint,
  // yaw_setpoint);

  // convert setpoints to degrees
  roll_setpoint = rad2deg(-1.0 * roll_setpoint);
  pitch_setpoint = rad2deg(-1.0 * pitch_setpoint);

  // printf("roll setpoint: %f\t", roll_setpoint);
  // printf("pitch_setpoint: %f\t\n", pitch_setpoint);
  // printf("target: ");
  // printf("x: %f\t", target(0));
  // printf("y: %f\t", target(1));
  // printf("z: %f\n", target(2));

  // set angle
  this->sbgc->setAngle(roll_setpoint, pitch_setpoint, 0);

  return 0;
}

int Gimbal::setGimbalAngles(double roll, double pitch, double yaw) {
  return this->sbgc->setAngle(roll, pitch, yaw);
}

}  // end of awesomo namespace
