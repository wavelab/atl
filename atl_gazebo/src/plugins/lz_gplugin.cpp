#include "wavesim_gazebo/plugins/lz_gplugin.hpp"


namespace wavesim {
namespace gaz {

LZGPlugin::LZGPlugin(void) {
  printf("LOADING [liblz_gplugin.so]!\n");
}

void LZGPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  gazebo::math::Pose pose;

  // clang-format off
  this->model = model;
  this->update_conn = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&LZGPlugin::onUpdate, this, _1)
  );
  // clang-format on

  // initialize robot
  pose = this->model->GetWorldPose();
  this->robot_states(0) = pose.pos.x;  // position x
  this->robot_states(1) = pose.pos.y;  // position y
  this->robot_states(2) = pose.rot.z;  // heading
  this->robot_inputs(0) = 0.0;         // velocity
  this->robot_inputs(1) = 0.0;         // angular velocity

  // gazebo node
  // clang-format off
  GazeboNode::configure();
  this->registerPublisher<gazebo::msgs::Pose>(POSE_GTOPIC);
  this->registerSubscriber(POSITION_SET_GTOPIC, &LZGPlugin::positionCallback, this);
  this->registerSubscriber(VELOCITY_SET_GTOPIC, &LZGPlugin::velocityCallback, this);
  this->registerSubscriber(ANGULAR_VEL_SET_GTOPIC, &LZGPlugin::angularVelocityCallback, this);
  // clang-format on
}

void LZGPlugin::onUpdate(const gazebo::common::UpdateInfo &info) {
  gazebo::math::Pose pose;
  gazebo::common::Time diff;
  double dt;
  Vec3 x, u, euler;
  Quaternion q;

  // calculate time step size
  diff = info.simTime - this->prev_sim_time;
  dt = diff.nsec / 1000000000.0;       // convert nsec to sec
  this->prev_sim_time = info.simTime;  // keep track of sim time

  // update robot
  x = this->robot_states;
  u = this->robot_inputs;
  x(0) = x(0) + u(0) * cos(x(2)) * dt;
  x(1) = x(1) + u(0) * sin(x(2)) * dt;
  x(2) = x(2) + u(1) * dt;

  // limit bearing to be within 0 - 360
  if (x(2) > deg2rad(360.0)) {
    x(2) -= deg2rad(360.0);
  } else if (x(2) < 0.0) {
    x(2) += deg2rad(360.0);
  }

  // set robot state
  this->robot_states = x;

  // set robot pose
  pose = this->model->GetWorldPose();
  pose.pos.x = this->robot_states(0);
  pose.pos.y = this->robot_states(1);

  // make yaw be between +/- 180.0
  euler << 0.0, 0.0, this->robot_states(2);
  euler(2) = fmod(euler(2) + deg2rad(180.0), deg2rad(360.0)) - deg2rad(180.0);

  // convert to quaternion
  euler2quat(euler, 321, q);
  pose.rot.z = q.z();

  this->model->SetWorldPose(pose);
  this->publishPose();
}

void LZGPlugin::publishPose(void) {
  gazebo::msgs::Pose msg;
  gazebo::math::Pose pose;

  pose = this->model->GetWorldPose();
  msg.mutable_position()->set_x(pose.pos.x);
  msg.mutable_position()->set_y(pose.pos.y);
  msg.mutable_position()->set_z(pose.pos.z);
  msg.mutable_orientation()->set_w(pose.rot.w);
  msg.mutable_orientation()->set_x(pose.rot.x);
  msg.mutable_orientation()->set_y(pose.rot.y);
  msg.mutable_orientation()->set_z(pose.rot.z);

  this->gaz_pubs[POSE_GTOPIC]->Publish(msg);
}

void LZGPlugin::positionCallback(ConstVector3dPtr &msg) {
  gazebo::math::Pose pose;

  // parse msg and set robot position in model
  this->robot_states(0) = msg->x();
  this->robot_states(1) = msg->y();

  // set robot position in gazebo
  pose = this->model->GetWorldPose();
  pose.pos.x = this->robot_states(0);
  pose.pos.y = this->robot_states(1);
  this->model->SetWorldPose(pose);
}

void LZGPlugin::velocityCallback(ConstAnyPtr &msg) {
  if (msg->type() == gazebo::msgs::Any::DOUBLE) {
    this->robot_inputs(0) = msg->double_value();
  }
}

void LZGPlugin::angularVelocityCallback(ConstAnyPtr &msg) {
  if (msg->type() == gazebo::msgs::Any::DOUBLE) {
    this->robot_inputs(1) = msg->double_value();
  }
}

GZ_REGISTER_MODEL_PLUGIN(LZGPlugin)
}  // end of gaz namespace
}  // end of wavesim namespace
