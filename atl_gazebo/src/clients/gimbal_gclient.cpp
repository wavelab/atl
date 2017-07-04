#include "atl_gazebo/clients/gimbal_gclient.hpp"


namespace atl {
namespace gaz {

GimbalGClient::GimbalGClient(void) {
  this->connected = false;
  this->frame_orientation = Quaternion();
  this->joint_orientation = Quaternion();
}

GimbalGClient::~GimbalGClient(void) {
  if (this->connected) {
    gazebo::client::shutdown();
  }
}

int GimbalGClient::configure(void) {
  // pre-check
  this->connected = gazebo::client::setup(0, NULL);
  if (this->connected == false) {
    return -1;
  }

  // setup gazebo node
  // clang-format off
  GazeboNode::configure();
  this->registerPublisher<VEC3_MSG>(SETPOINT_GTOPIC);
  this->registerPublisher<VEC3_MSG>(TRACK_GTOPIC);
  this->registerSubscriber(FRAME_ORIENTATION_GTOPIC, &GimbalGClient::frameOrientationCallback, this);
  this->registerSubscriber(JOINT_ORIENTATION_GTOPIC, &GimbalGClient::jointOrientationCallback, this);
  this->waitForConnection();
  // clang-format on

  return 0;
}

void GimbalGClient::setAttitude(Vec3 euler_if) {
  gazebo::msgs::Vector3d msg;

  msg.set_x(euler_if(0));
  msg.set_y(euler_if(1));
  msg.set_z(euler_if(2));

  this->gaz_pubs[SETPOINT_GTOPIC]->Publish(msg);
}

void GimbalGClient::trackTarget(Vec3 target_cf) {
  gazebo::msgs::Vector3d msg;

  msg.set_x(target_cf(0));
  msg.set_y(target_cf(1));
  msg.set_z(target_cf(2));

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

}  // namespace gaz
}  // namespace atl
