#include "atl/vision/gimbal/gimbal.hpp"

namespace atl {

Gimbal::Gimbal() {
  this->configured = false;

  this->sbgc = SBGC();
  this->camera_offset = Pose();
  std::fill_n(this->limits, 6, 0);
  this->enable_tracking = false;

  this->setpoints = Vec3();
  this->target_bpf = Vec3();

  imu_accel = Vec3();
  imu_gyro = Vec3();
  camera_angles = Vec3();
  frame_angles = Vec3();
  rc_angles = Vec3();
  encoder_angles = Vec3();
}

Gimbal::~Gimbal() { this->off(); }

int Gimbal::configure(std::string config_file) {
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
  this->camera_offset = Pose(roll, pitch, yaw, x, y, z);

  // gimbal limits
  for (int i = 0; i < 6; i++) {
    this->limits[i] = deg2rad(this->limits[i]);
  }

  this->configured = true;
  return 0;
}

int Gimbal::on() { return this->sbgc.on(); }

int Gimbal::off() { return this->sbgc.off(); }

Vec3 Gimbal::getTargetInBF(Pose camera_offset, Vec3 target_cf) {
  Vec3 target_nwu;
  Mat3 R;
  Vec3 t;

  // transform camera frame to gimbal joint frame (NWU)
  cf2nwu(target_cf, target_nwu);

  // camera mount offset
  R = camera_offset.rotationMatrix();
  t = camera_offset.position;

  // transform target from camera frame to gimbal joint frame
  return R * target_nwu + t;
}

Vec3 Gimbal::getTargetInBPF(Pose camera_offset,
                            Vec3 target_cf,
                            Quaternion joint_if) {
  Vec3 p;
  Mat3 R;

  // joint is assumed to be NWU frame (same as ROS REP-103)
  R = joint_if.toRotationMatrix();

  // transform target in camera frame to body frame
  p = Gimbal::getTargetInBF(camera_offset, target_cf);

  // transform target in camera frame to body planar frame
  return R * p;
}

Vec3 Gimbal::getTargetInIF(Vec3 target_bpf,
                           Vec3 gimbal_position,
                           Quaternion gimbal_frame_if) {
  Vec3 euler, target, target_enu, target_if;
  Mat3 R;

  // filter out roll and pitch in quaternion
  quat2euler(gimbal_frame_if, 321, euler);
  euler << 0.0, 0.0, euler(2);
  euler2rot(euler, 321, R);

  // compensate yaw in target from body planar frame to inertial frame
  target = R * target_bpf;

  // convert target from NWU to ENU
  nwu2enu(target, target_enu);

  // transform target from body to inertial frame
  target_if = gimbal_position + target_enu;

  return target_if;
}

int Gimbal::getTargetInBPF(Vec3 target_cf, Vec3 &target_bpf) {
  int retval;
  Vec3 tmp, euler;
  Quaternion gimbal_imu;

  // get data from SimpleBGC
  retval = this->sbgc.getRealtimeData();
  if (retval != 0) {
    return -1;
  }

  // convert sbgc gimbal angle to quaternion
  // (making gimbal angles NED, notice the -ve sign in pitch)
  euler(0) = deg2rad(this->sbgc.data.camera_angles(0));
  euler(1) = deg2rad(this->sbgc.data.camera_angles(1));
  euler(2) = 0.0;
  euler2quat(euler, 321, gimbal_imu);

  // camera frame to camera mount frame
  tmp = this->camera_offset.rotationMatrix().inverse() * target_cf;
  // inverse because we want tag relative to quad
  // without it, results are relative to tag

  // camera mount frame to body planar frame
  tmp = gimbal_imu.toRotationMatrix() * tmp;
  target_bpf(0) = tmp(0);
  target_bpf(1) = tmp(1);
  target_bpf(2) = tmp(2);
  this->target_bpf = target_bpf;

  return 0;
}

int Gimbal::trackTarget(Vec3 target_bpf) {
  double dist;

  // pre-check
  if (this->enable_tracking == false) {
    return 0;
  }

  // calculate roll pitch yaw setpoints
  // Note: setpoints are assuming Gimbal are in NWU frame
  // NWU frame: (x - forward, y - left, z - up)
  dist = target_bpf.norm();
  this->setpoints(0) = asin(target_bpf(1) / dist); // roll setpoint
  this->setpoints(1) = asin(target_bpf(0) / dist); // pitch setpoint
  this->setpoints(2) = 0.0; // yaw setpoint - unsupported at the moment

  return 0;
}

int Gimbal::updateGimbalStates() {
  int retval;
  const double k_gravity = 9.80665;

  retval = this->sbgc.getRealtimeData4();
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

  return this->sbgc.setAngle(
      this->setpoints(0), this->setpoints(1), this->setpoints(2));
}

void Gimbal::printSetpoints() {
  std::cout << "roll setpoint: " << this->setpoints(0) << "\t";
  std::cout << "pitch setpoint: " << this->setpoints(1) << "\t";
  std::cout << "target: [";
  std::cout << "x: " << this->target_bpf(0) << "\t";
  std::cout << "y: " << this->target_bpf(1) << "\t";
  std::cout << "z: " << this->target_bpf(2);
  std::cout << "]" << std::endl;
}

} // namespace atl
