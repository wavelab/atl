#include "atl/gazebo/clients/gimbal_gclient.hpp"

namespace atl {
namespace gaz {

GimbalGClient::GimbalGClient() {
  this->connected = false;
  this->frame_orientation = Quaternion();
  this->joint_orientation = Quaternion();
}

GimbalGClient::~GimbalGClient() {
  if (this->connected) {
    gazebo::client::shutdown();
  }
}

int GimbalGClient::configure() {
  // pre-check
  this->connected = gazebo::client::setup(0, NULL);
  if (this->connected == false) {
    return -1;
  }

  // setup gazebo node
  // clang-format off
  GazeboNode::configure();
  this->addPublisher<VEC3_MSG>(SETPOINT_GTOPIC);
  this->addPublisher<VEC3_MSG>(TRACK_GTOPIC);
  this->addSubscriber(FRAME_ORIENTATION_GTOPIC, &GimbalGClient::frameOrientationCallback, this);
  this->addSubscriber(JOINT_ORIENTATION_GTOPIC, &GimbalGClient::jointOrientationCallback, this);
  this->waitForConnection();
  // clang-format on

  return 0;
}

void GimbalGClient::setAttitude(Vec3 euler_W) {
  gazebo::msgs::Vector3d msg;

  msg.set_x(euler_W(0));
  msg.set_y(euler_W(1));
  msg.set_z(euler_W(2));

  this->gaz_pubs[SETPOINT_GTOPIC]->Publish(msg);
}

void GimbalGClient::trackTarget(Vec3 target_C) {
  gazebo::msgs::Vector3d msg;

  msg.set_x(target_C(0));
  msg.set_y(target_C(1));
  msg.set_z(target_C(2));

  this->gaz_pubs[TRACK_GTOPIC]->Publish(msg);
}

void GimbalGClient::frameOrientationCallback(ConstQuaternionPtr &msg) {
  this->frame_orientation.w() = msg->w();
  this->frame_orientation.x() = msg->x();
  this->frame_orientation.y() = msg->y();
  this->frame_orientation.z() = msg->z();
}

void GimbalGClient::jointOrientationCallback(ConstQuaternionPtr &msg) {
  this->joint_orientation.w() = msg->w();
  this->joint_orientation.x() = msg->x();
  this->joint_orientation.y() = msg->y();
  this->joint_orientation.z() = msg->z();
}

} // namespace gaz
} // namespace atl
