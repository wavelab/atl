#include "atl/gazebo/clients/quadrotor_gclient.hpp"

namespace atl {
namespace gaz {

int QuadrotorGClient::configure() {
  // pre-check
  this->connected = gazebo::client::setup(0, NULL);
  if (this->connected == false) {
    return -1;
  }

  // initialize data
  this->position = VecX::Zero(3);
  this->orientation = Quaternion{1.0, 0.0, 0.0, 0.0};

  // setup gazebo node
  GazeboNode::configure();

  // subscribers
  // clang-format off
  this->registerSubscriber(POSE_GTOPIC, &QuadrotorGClient::poseCallback, this);
  this->registerSubscriber(VELOCITY_GTOPIC, &QuadrotorGClient::velocityCallback, this);
  // clang-format on

  // publishers
  this->registerPublisher<ATT_SETPOINT_MSG>(ATT_SETPOINT_GTOPIC);
  this->registerPublisher<POS_SETPOINT_MSG>(POS_SETPOINT_GTOPIC);
  this->registerPublisher<VEL_SETPOINT_MSG>(VEL_SETPOINT_GTOPIC);
  this->waitForConnection();

  return 0;
}

void QuadrotorGClient::poseCallback(ConstPosePtr &msg) {
  // position
  this->position(0) = msg->position().x();
  this->position(1) = msg->position().y();
  this->position(2) = msg->position().z();

  // orientation
  this->orientation.w() = msg->orientation().w();
  this->orientation.x() = msg->orientation().x();
  this->orientation.y() = msg->orientation().y();
  this->orientation.z() = msg->orientation().z();
}

void QuadrotorGClient::velocityCallback(ConstVector3dPtr &msg) {
  this->velocity(0) = msg->x();
  this->velocity(1) = msg->y();
  this->velocity(2) = msg->z();
}

int QuadrotorGClient::setAttitude(double r, double p, double y, double t) {
  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  ATT_SETPOINT_MSG msg;

  msg.set_roll(r);
  msg.set_pitch(p);
  msg.set_yaw(y);
  msg.set_throttle(t);

  this->attitude_setpoints << r, p, y, t;
  this->gaz_pubs[ATT_SETPOINT_GTOPIC]->Publish(msg);

  return 0;
}

int QuadrotorGClient::setPosition(double x, double y, double z) {
  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  POS_SETPOINT_MSG msg;

  msg.set_x(x);
  msg.set_y(y);
  msg.set_z(z);

  this->position_setpoints << x, y, z;
  this->gaz_pubs[POS_SETPOINT_GTOPIC]->Publish(msg);

  return 0;
}

int QuadrotorGClient::setVelocity(double vx, double vy, double vz) {
  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  VEL_SETPOINT_MSG msg;

  msg.set_vx(vx);
  msg.set_vy(vy);
  msg.set_vz(vz);

  this->velocity_setpoints << vx, vy, vz;
  this->gaz_pubs[VEL_SETPOINT_GTOPIC]->Publish(msg);

  return 0;
}

}  // namespace gaz
}  // namespace atl
