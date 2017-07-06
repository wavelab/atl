#include "atl/gazebo/plugins/lz_gplugin.hpp"


namespace atl {
namespace gaz {

LZGPlugin::LZGPlugin(void) {
  printf("LOADING [liblz_gplugin.so]!\n");
}

void LZGPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  UNUSED(sdf);

  // clang-format off
  this->model = model;
  this->update_conn = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&LZGPlugin::onUpdate, this, _1)
  );
  // clang-format on

  // initialize robot
  ignition::math::Pose3d pose = this->model->WorldPose();
  this->robot_states(0) = pose.Pos().X();  // position x
  this->robot_states(1) = pose.Pos().Y();  // position y
  this->robot_states(2) = pose.Rot().Z();  // heading
  this->robot_inputs(0) = 0.0;             // velocity
  this->robot_inputs(1) = 0.0;             // angular velocity

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
  ignition::math::Pose3d pose = this->model->WorldPose();
  pose.Pos().X() = this->robot_states(0);
  pose.Pos().Y() = this->robot_states(1);

  // make yaw be between +/- 180.0
  euler << 0.0, 0.0, this->robot_states(2);
  euler(2) = fmod(euler(2) + deg2rad(180.0), deg2rad(360.0)) - deg2rad(180.0);

  // convert to quaternion
  euler2quat(euler, 321, q);
  pose.Rot().Z() = q.z();

  this->model->SetWorldPose(pose);
  this->publishPose();
}

void LZGPlugin::publishPose(void) {
  ignition::math::Pose3d pose = this->model->WorldPose();

  gazebo::msgs::Pose msg;
  msg.mutable_position()->set_x(pose.Pos().X());
  msg.mutable_position()->set_y(pose.Pos().Y());
  msg.mutable_position()->set_z(pose.Pos().Z());
  msg.mutable_orientation()->set_w(pose.Rot().W());
  msg.mutable_orientation()->set_x(pose.Rot().X());
  msg.mutable_orientation()->set_y(pose.Rot().Y());
  msg.mutable_orientation()->set_z(pose.Rot().Z());

  this->gaz_pubs[POSE_GTOPIC]->Publish(msg);
}

void LZGPlugin::positionCallback(ConstVector3dPtr &msg) {
  // parse msg and set robot position in model
  this->robot_states(0) = msg->x();
  this->robot_states(1) = msg->y();

  // set robot position in gazebo
  ignition::math::Pose3d pose = this->model->WorldPose();
  pose.Pos().X() = this->robot_states(0);
  pose.Pos().Y() = this->robot_states(1);
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
}  // namespace gaz
}  // namespace atl
