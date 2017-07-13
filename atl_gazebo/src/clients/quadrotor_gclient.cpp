#include "atl/gazebo/clients/quadrotor_gclient.hpp"

namespace atl {
namespace gaz {

QuadrotorGClient::QuadrotorGClient() {
  this->connected = false;

  this->pose = VecX(6);
  this->velocity << 0.0, 0.0, 0.0;
  this->attitude_setpoints << 0.0, 0.0, 0.0, 0.0;
  this->position_setpoints << 0.0, 0.0, 0.0;
  this->velocity_setpoints << 0.0, 0.0, 0.0;
}

QuadrotorGClient::~QuadrotorGClient() {
  if (this->connected) {
    gazebo::client::shutdown();
  }
}

int QuadrotorGClient::configure() {
  // pre-check
  this->connected = gazebo::client::setup(0, NULL);
  if (this->connected == false) {
    return -1;
  }

  // initialize data
  this->pose = VecX(6);

  // setup gazebo node
  GazeboNode::configure();

  // subscribers
  // clang-format off
  this->registerSubscriber(POSE_GTOPIC, &QuadrotorGClient::poseCallback, this);
  this->registerSubscriber(VELOCITY_GTOPIC, &QuadrotorGClient::velocityCallback, this);
  // clang-format on

  // publishers
  // clang-format off
  this->registerPublisher<ATT_SETPOINT_MSG>(ATT_SETPOINT_GTOPIC);
  this->registerPublisher<POS_SETPOINT_MSG>(POS_SETPOINT_GTOPIC);
  this->registerPublisher<VEL_SETPOINT_MSG>(VEL_SETPOINT_GTOPIC);
  this->waitForConnection();
  // clang-format on

  // follow
  gazebo::gui::Events::follow("quadrotor");

  return 0;
}

void QuadrotorGClient::poseCallback(RPYPosePtr &msg) {
  // position
  this->pose(0) = msg->x();
  this->pose(1) = msg->y();
  this->pose(2) = msg->z();

  // orientation
  this->pose(3) = msg->roll();
  this->pose(4) = msg->pitch();
  this->pose(5) = msg->yaw();
}

void QuadrotorGClient::velocityCallback(ConstVector3dPtr &msg) {
  this->velocity(0) = msg->x();
  this->velocity(1) = msg->y();
  this->velocity(2) = msg->z();
}

int QuadrotorGClient::setAttitude(double r, double p, double y, double t) {
  ATT_SETPOINT_MSG msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.set_roll(r);
  msg.set_pitch(p);
  msg.set_yaw(y);
  msg.set_throttle(t);
  this->attitude_setpoints << r, p, y, t;
  this->gaz_pubs[ATT_SETPOINT_GTOPIC]->Publish(msg);

  return 0;
}

int QuadrotorGClient::setPosition(double x, double y, double z) {
  POS_SETPOINT_MSG msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.set_x(x);
  msg.set_y(y);
  msg.set_z(z);
  this->position_setpoints << x, y, z;
  this->gaz_pubs[POS_SETPOINT_GTOPIC]->Publish(msg);

  return 0;
}

int QuadrotorGClient::setVelocity(double vx, double vy, double vz) {
  VEL_SETPOINT_MSG msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.set_vx(vx);
  msg.set_vy(vy);
  msg.set_vz(vz);
  this->velocity_setpoints << vx, vy, vz;
  this->gaz_pubs[VEL_SETPOINT_GTOPIC]->Publish(msg);

  return 0;
}

}  // namespace gaz
}  // namespace atl
