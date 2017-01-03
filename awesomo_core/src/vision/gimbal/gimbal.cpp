#include "awesomo_core/vision/gimbal/gimbal.hpp"


namespace awesomo {

Gimbal::Gimbal(void) {
  this->configured = false;

  this->sbgc = SBGC();
  this->camera_offset = Pose();
  std::fill_n(this->limits, 6, 0);
  this->setpoints = Vec3();
  this->target_bpf = Vec3();
}

int Gimbal::configure(std::string config_file) {
  std::string device_path;
  ConfigParser parser;
  double roll, pitch, yaw;
  double x, y, z;

  // parse config file
  parser.addParam<std::string>("device_path", &device_path);
  parser.addParam<double>("camera_offset.roll", &roll);
  parser.addParam<double>("camera_offset.pitch", &pitch);
  parser.addParam<double>("camera_offset.yaw", &yaw);
  parser.addParam<double>("camera_offset.x", &x);
  parser.addParam<double>("camera_offset.y", &y);
  parser.addParam<double>("camera_offset.z", &z);
  parser.addParam<double>("gimbal_limits.roll_min", &this->limits[0]);
  parser.addParam<double>("gimbal_limits.roll_max", &this->limits[1]);
  parser.addParam<double>("gimbal_limits.pitch_min", &this->limits[2]);
  parser.addParam<double>("gimbal_limits.pitch_max", &this->limits[3]);
  parser.addParam<double>("gimbal_limits.yaw_min", &this->limits[4]);
  parser.addParam<double>("gimbal_limits.yaw_max", &this->limits[5]);
  if (parser.load(config_file) != 0) {
    log_err("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // SimpleBGC serial connection
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

Vec3 Gimbal::getTargetPositionInBodyFrame(Vec3 target_cf) {
  Vec3 target_nwu;
  Mat3 R;
  Vec3 t;

  // transform camera frame to NWU frame
  // camera frame:  (z - forward, x - right, y - down)
  // NWU frame:  (x - forward, y - left, z - up)
  target_nwu(0) = target_cf(2);
  target_nwu(1) = -target_cf(0);
  target_nwu(2) = -target_cf(1);

  // camera mount offset
  R = this->camera_offset.rotationMatrix();
  t = this->camera_offset.position;

  // transform target from camera frame to body frame
  return (R * target_nwu + t);
}

Vec3 Gimbal::getTargetPositionInBodyPlanarFrame(Vec3 target_cf,
                                                Quaternion &imu_if) {
  Vec3 p;
  Mat3 R;

  // imu is assumed to be NWU frame (same as ROS REP-103)
  R = imu_if.toRotationMatrix();

  // transform target in camera frame to body frame
  p = this->getTargetPositionInBodyFrame(target_cf);

  // transform target in camera frame to body planar frame
  this->target_bpf = R * p;

  return this->target_bpf;
}

int Gimbal::getTargetPositionInBodyPlanarFrame(Vec3 target_cf,
                                               Vec3 &target_bpf) {
  int retval;
  Vec3 tmp;
  Quaternion gimbal_imu;

  // get data from SimpleBGC
  retval = this->sbgc.getRealtimeData();
  if (retval != 0) {
    return -1;
  }

  // convert sbgc gimbal angle to quaternion
  // (making gimbal angles NED, notice the -ve sign in pitch)
  euler2Quaternion(deg2rad(this->sbgc.data.camera_angles(0)),
                   deg2rad(-this->sbgc.data.camera_angles(1)),
                   0.0,
                   gimbal_imu);

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

  // calculate roll pitch yaw setpoints
  dist = target_bpf.norm();
  this->setpoints(0) = asin(target_bpf(1) / dist);  // roll setpoint
  this->setpoints(1) = asin(target_bpf(0) / dist);  // pitch setpoint
  this->setpoints(2) = 0.0;  // yaw setpoint - unsupported at the moment
  // Note: setpoints are assuming Gimbal are in NWU frame
  // NWU frame: (x - forward, y - left, z - up)

  // // check setpoints
  // for (int i = 0; i < 3; i++) {
  //   // limit setpoints
  //   if (this->setpoints(i) < this->limits[i * 2]) {
  //     this->setpoints(i) = this->limits[i * 2];
  //   } else if (this->setpoints(i) > this->limits[(i * 2) + 1]) {
  //     this->setpoints(i) = this->limits[(i * 2) + 1];
  //   }
  // }
  //
  // // set angle
  // this->setAngle(this->setpoints(0), this->setpoints(1));

  return 0;
}

int Gimbal::setAngle(double roll, double pitch) {
  this->setpoints(0) = roll;
  this->setpoints(1) = pitch;
  this->setpoints(2) = 0.0;

  return this->sbgc.setAngle(roll, pitch, 0.0);
}

void Gimbal::printSetpoints(void) {
  std::cout << "roll setpoint: " << this->setpoints(0) << "\t";
  std::cout << "pitch setpoint: " << this->setpoints(1) << "\t";
  std::cout << "target: [";
  std::cout << "x: " << this->target_bpf(0) << "\t";
  std::cout << "y: " << this->target_bpf(1) << "\t";
  std::cout << "z: " << this->target_bpf(2);
  std::cout << "]" << std::endl;
}

}  // end of awesomo namespace
