#include "atl/gazebo/clients/lz_gclient.hpp"

namespace atl {
namespace gaz {

LZGClient::LZGClient() { this->connected = false; }

LZGClient::~LZGClient() {
  if (this->connected) {
    gazebo::client::shutdown();
  }
}

int LZGClient::configure() {
  // pre-check
  this->connected = gazebo::client::setup(0, NULL);
  if (this->connected == false) {
    return -1;
  }

  // setup gazebo node
  // clang-format off
  GazeboNode::configure();
  this->registerSubscriber(POSE_GTOPIC, &LZGClient::poseCallback, this);
  this->registerPublisher<gazebo::msgs::Vector3d>(POSITION_SET_GTOPIC);
  this->registerPublisher<gazebo::msgs::Any>(VELOCITY_SET_GTOPIC);
  this->registerPublisher<gazebo::msgs::Any>(ANGULAR_VEL_SET_GTOPIC);
  this->waitForConnection();
  // clang-format on

  return 0;
}

void LZGClient::poseCallback(ConstPosePtr &msg) {
  this->pose.Pos().X() = msg->position().x();
  this->pose.Pos().Y() = msg->position().y();
  this->pose.Pos().Z() = msg->position().z();

  this->pose.Rot().W() = msg->orientation().w();
  this->pose.Rot().X() = msg->orientation().x();
  this->pose.Rot().Y() = msg->orientation().y();
  this->pose.Rot().Z() = msg->orientation().z();
}

void LZGClient::setXYPosition(double x, double y) {
  gazebo::msgs::Vector3d msg;
  msg.set_x(x);
  msg.set_y(y);
  msg.set_z(0.0);
  this->gaz_pubs[POSITION_SET_GTOPIC]->Publish(msg);
}

void LZGClient::setVelocity(double vel) {
  gazebo::msgs::Any msg;
  msg.set_type(gazebo::msgs::Any::DOUBLE);
  msg.set_double_value(vel);
  this->gaz_pubs[VELOCITY_SET_GTOPIC]->Publish(msg);
}

void LZGClient::setAngularVelocity(double ang_vel) {
  gazebo::msgs::Any msg;

  msg.set_type(gazebo::msgs::Any::DOUBLE);
  msg.set_double_value(ang_vel);
  this->gaz_pubs[ANGULAR_VEL_SET_GTOPIC]->Publish(msg);
}

} // namespace gaz
} // namespace atl
