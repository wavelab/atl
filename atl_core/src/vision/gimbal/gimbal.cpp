#include "atl/vision/gimbal/gimbal.hpp"

namespace atl {

int Gimbal::configure(const std::string &config_file) {
  std::string device_path;
  ConfigParser parser;
  double roll, pitch, yaw;
  double x, y, z;

  // parse config file
  parser.addParam("device_path", &device_path);
  parser.addParam("enable_tracking", &this->enable_tracking);
  parser.addParam("camera_offset.roll", &roll);
  parser.addParam("camera_offset.pitch", &pitch);
  parser.addParam("camera_offset.yaw", &yaw);
  parser.addParam("camera_offset.x", &x);
  parser.addParam("camera_offset.y", &y);
  parser.addParam("camera_offset.z", &z);
  parser.addParam("gimbal_limits.roll_min", &this->limits[0]);
  parser.addParam("gimbal_limits.roll_max", &this->limits[1]);
  parser.addParam("gimbal_limits.pitch_min", &this->limits[2]);
  parser.addParam("gimbal_limits.pitch_max", &this->limits[3]);
  parser.addParam("gimbal_limits.yaw_min", &this->limits[4]);
  parser.addParam("gimbal_limits.yaw_max", &this->limits[5]);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // simple bgc serial connection
  this->sbgc = SBGC(device_path);
  if (this->sbgc.connect() != 0 || this->sbgc.on() != 0) {
    return -2;
  }

  // camera mount offsets
  this->camera_offset = Pose(BODY_FRAME, roll, pitch, yaw, x, y, z);

  // gimbal limits
  for (int i = 0; i < 6; i++) {
    this->limits[i] = deg2rad(this->limits[i]);
  }

  this->configured = true;
  return 0;
}

int Gimbal::on() { return this->sbgc.on(); }

int Gimbal::off() { return this->sbgc.off(); }

Vec3 Gimbal::getTargetInBF(const Pose &camera_offset, const Vec3 &target_C) {
  // camera mount offset
  Mat3 R{camera_offset.rotationMatrix()};
  Vec3 t{camera_offset.position};

  // transform target in camera frame -> world frame -> body frame
  return T_B_W{R, t} * T_W_C * target_C;
}

Vec3 Gimbal::getTargetInBPF(const Pose &camera_offset,
                            const Vec3 &target_C,
                            const Quaternion &joint_W) {
  // joint is assumed to be NWU frame (same as ROS REP-103)
  Mat3 R = joint_W.toRotationMatrix();

  // transform target in camera frame to body frame
  Vec3 p = Gimbal::getTargetInBF(camera_offset, target_C);

  // transform target in camera frame to body planar frame
  return R * p;
}

Vec3 Gimbal::getTargetInIF(const Vec3 &target_P,
                           const Vec3 &gimbal_position,
                           const Quaternion &gimbal_frame_W) {
  // filter out roll and pitch in quaternion
  Vec3 euler = quatToEuler321(gimbal_frame_W);
  euler << 0.0, 0.0, euler(2);
  Mat3 R = euler321ToRot(euler);

  // compensate yaw in target from body planar frame to inertial frame
  return (R * target_P) + gimbal_position;
}

int Gimbal::getTargetInBPF(const Vec3 &target_C, Vec3 &target_P) {
  Vec3 tmp;

  // get data from SimpleBGC
  int retval = this->sbgc.getRealtimeData();
  if (retval != 0) {
    return -1;
  }

  // convert sbgc gimbal angle to quaternion
  Vec3 euler{deg2rad(this->sbgc.data.camera_angles(0)),
             deg2rad(this->sbgc.data.camera_angles(1)),
             0.0};
  Quaternion gimbal_imu = euler321ToQuat(euler);

  // camera frame to camera mount frame
  tmp = this->camera_offset.rotationMatrix().inverse() * target_C;
  // inverse because we want tag relative to quad
  // without it, results are relative to tag

  // camera mount frame to body planar frame
  tmp = gimbal_imu.toRotationMatrix() * tmp;
  target_P(0) = tmp(0);
  target_P(1) = tmp(1);
  target_P(2) = tmp(2);
  this->target_P = target_P;

  return 0;
}

int Gimbal::trackTarget(const Vec3 &target_P) {
  // pre-check
  if (this->enable_tracking == false) {
    return 0;
  }

  // calculate roll pitch yaw setpoints
  // Note: setpoints are assuming Gimbal are in NWU frame
  // NWU frame: (x - forward, y - left, z - up)
  const double dist = target_P.norm();
  this->setpoints(0) = asin(target_P(1) / dist); // roll setpoint
  this->setpoints(1) = asin(target_P(0) / dist); // pitch setpoint
  this->setpoints(2) = 0.0; // yaw setpoint - unsupported at the moment

  return 0;
}

int Gimbal::updateGimbalStates() {
  const double k_gravity = 9.80665;

  int retval = this->sbgc.getRealtimeData4();
  if (retval != 0) {
    return -1;
  }

  // retval = this->sbgc.getAnglesExt();
  // if (retval != 0) {
  //   return -1;
  // }

  // convert from G's to m/s^2
  this->imu_accel(0) = this->sbgc.data.accel(0) * k_gravity;
  this->imu_accel(1) = this->sbgc.data.accel(1) * k_gravity;
  this->imu_accel(2) = this->sbgc.data.accel(2) * k_gravity;

  this->imu_gyro(0) = deg2rad(this->sbgc.data.gyro(0));
  this->imu_gyro(1) = deg2rad(this->sbgc.data.gyro(1));
  this->imu_gyro(2) = deg2rad(this->sbgc.data.gyro(2));

  this->camera_angles(0) = deg2rad(this->sbgc.data.camera_angles(0));
  this->camera_angles(1) = deg2rad(this->sbgc.data.camera_angles(1));
  this->camera_angles(2) = deg2rad(this->sbgc.data.camera_angles(2));

  this->frame_angles(0) = deg2rad(this->sbgc.data.frame_angles(0));
  this->frame_angles(1) = deg2rad(this->sbgc.data.frame_angles(1));
  this->frame_angles(2) = deg2rad(this->sbgc.data.frame_angles(2));

  this->rc_angles(0) = deg2rad(this->sbgc.data.rc_angles(0));
  this->rc_angles(1) = deg2rad(this->sbgc.data.rc_angles(1));
  this->rc_angles(2) = deg2rad(this->sbgc.data.rc_angles(2));

  this->encoder_angles(0) = deg2rad(this->sbgc.data.encoder_angles(0));
  this->encoder_angles(1) = deg2rad(this->sbgc.data.encoder_angles(1));
  this->encoder_angles(2) = deg2rad(this->sbgc.data.encoder_angles(2));

  return 0;
}

int Gimbal::setAngle(double roll, double pitch) {
  // pre-check
  if (this->enable_tracking == false) {
    return 0;
  }

  this->setpoints(0) = roll * 180 / M_PI;
  this->setpoints(1) = pitch * 180 / M_PI;
  this->setpoints(2) = 0.0 * 180 / M_PI;

  return this->sbgc.setAngle(this->setpoints(0),
                             this->setpoints(1),
                             this->setpoints(2));
}

void Gimbal::printSetpoints() {
  std::cout << "roll setpoint: " << this->setpoints(0) << "\t";
  std::cout << "pitch setpoint: " << this->setpoints(1) << "\t";
  std::cout << "target: [";
  std::cout << "x: " << this->target_P(0) << "\t";
  std::cout << "y: " << this->target_P(1) << "\t";
  std::cout << "z: " << this->target_P(2);
  std::cout << "]" << std::endl;
}

} // namespace atl
