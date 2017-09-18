#include "atl/models/gimbal.hpp"

namespace atl {

// ATTITUDE CONTROLLER
Vec2 Gimbal2AxisController::update(Vec2 setpoints, Vec2 actual, double dt) {
  // limit rate to 100Hz
  // increment coUNTer and only take action when 0.001s accumulates
  this->dt += dt;
  if (this->dt >= 0.001) {
    // time to update; reset the counter
    this->dt = 0.0;

    // calculate roll and pitch outputs using independent controllers
    auto roll = this->roll_controller.update(setpoints(0), actual(0), dt);
    auto pitch = this->pitch_controller.update(setpoints(1), actual(1), dt);

    // keep track of outputs
    this->outputs << roll, pitch;
  }
  return this->outputs;
}

// GIMBAL MODEL
void Gimbal2AxisModel::update(Vec2 motor_inputs, double dt) {
  // update states
  float ph = this->states(0);
  float ph_vel = this->states(1);
  float th = this->states(2);
  float th_vel = this->states(3);

  this->states(0) = ph + ph_vel * dt;
  this->states(1) = ph_vel + (motor_inputs(0) / this->Ix) * dt;
  this->states(2) = th + th_vel * dt;
  this->states(3) = th_vel + (motor_inputs(1) / this->Ix) * dt;

  // update joint orientation
  Vec3 euler, target;
  euler << this->states(0), this->states(2), 0.0;
  this->joint_orientation = euler321ToQuat(euler);

  // track target attitude
  euler = quatToEuler321(this->frame_orientation);
  this->joint_setpoints(0) = target_attitude_W(0) - euler(0);
  this->joint_setpoints(1) = target_attitude_W(1) - euler(1);
}

Vec2 Gimbal2AxisModel::attitudeControllerControl(double dt) {
  Vec2 actual_attitude{this->states(0), this->states(2)};
  return this->joint_controller.update(this->joint_setpoints,
                                       actual_attitude,
                                       dt);
}

void Gimbal2AxisModel::setFrameOrientation(Quaternion frame_W) {
  // filter out yaw - we do not need it
  Vec3 euler = quatToEuler321(frame_W);
  euler(2) = 0.0;

  // set gimbal frame orientation
  this->frame_orientation = euler321ToQuat(euler);
}

void Gimbal2AxisModel::setAttitude(Vec2 euler_W) {
  this->target_attitude_W(0) = euler_W(0);
  this->target_attitude_W(1) = euler_W(1);
}

Vec3 Gimbal2AxisModel::getTargetInBF(Vec3 target_C) {
  Vec3 target_nwu;
  Mat3 R;
  Vec3 t;

  // transform camera frame to NWU frame
  // camera frame:  (z - forward, x - right, y - down)
  // NWU frame:  (x - forward, y - left, z - up)
  target_nwu(0) = target_C(2);
  target_nwu(1) = -target_C(0);
  target_nwu(2) = -target_C(1);

  // camera mount offset
  R = this->camera_offset.rotationMatrix();
  t = this->camera_offset.position;

  // transform target from camera frame to body frame
  return (R * target_nwu + t);
}

Vec3 Gimbal2AxisModel::getTargetInBPF(Vec3 target_C,
                                      Quaternion body_W,
                                      Quaternion joint_B) {
  // body is assumed to be NWU frame
  const Mat3 R_W = body_W.toRotationMatrix();

  // joint is assumed to be NWU frame
  const Mat3 R_B = joint_B.toRotationMatrix();

  // transform target in camera frame to body frame
  const Vec3 p = this->getTargetInBF(target_C);

  // transform target in camera frame to body planar frame
  const Vec3 target_P = R_W * R_B * p;

  return target_P;
}

void Gimbal2AxisModel::trackTarget(Vec3 target_C) {
  double dist;
  Vec3 target;

  // obtain target in body planar frame
  target = this->getTargetInBPF(target_C,
                                this->frame_orientation,
                                this->joint_orientation);

  // update gimbal setpoints
  dist = target.norm();
  this->target_attitude_W(0) = asin(target(1) / dist);
  this->target_attitude_W(1) = -asin(target(0) / dist);
}

Vec4 Gimbal2AxisModel::getState() {
  Vec4 pose;

  pose(0) = this->states(0);
  pose(1) = this->states(1);
  pose(2) = this->states(2);
  pose(3) = this->states(3);

  return pose;
}

void Gimbal2AxisModel::printState() {
  std::cout << "roll: ";
  std::cout << std::setprecision(2) << this->states(0) << "\t";

  std::cout << "pitch: ";
  std::cout << std::setprecision(2) << this->states(2) << "\t";

  std::cout << "roll vel: ";
  std::cout << std::setprecision(2) << this->states(1) << "\t";

  std::cout << "pitch vel: ";
  std::cout << std::setprecision(2) << this->states(3) << std::endl;
}

} // namespace atl
