#include "atl/gazebo/plugins/quadrotor_gplugin.hpp"

namespace atl {
namespace gaz {

VecX pose2Vec(ignition::math::Pose3d pose) {
  ignition::math::Vector3d pos = pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  VecX eigen_pose(6);
  eigen_pose << pos.X(), pos.Y(), pos.Z(), rpy.X(), rpy.Y(), rpy.Z();

  return eigen_pose;
}

ignition::math::Pose3d vec2pose(VecX pose) {
  double x = pose(0);
  double y = pose(1);
  double z = pose(2);
  double phi = pose(3);
  double theta = pose(4);
  double psi = pose(5);
  return ignition::math::Pose3d(x, y, z, phi, theta, psi);
}

QuadrotorGPlugin::QuadrotorGPlugin() {
  printf("LOADING [libquadrotor_gplugin.so]!\n");
}

void QuadrotorGPlugin::Load(gazebo::physics::ModelPtr model,
                            sdf::ElementPtr sdf) {
  UNUSED(sdf);

  // set model and bind world update callback
  // clang-format off
  this->model = model;
  this->update_conn = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&QuadrotorGPlugin::onUpdate, this, _1)
  );
  // clang-format on

  // set quadrotor model initial pose
  this->model->SetGravityMode(false);
  ignition::math::Pose3d pose = this->model->WorldPose();

  VecX quad_pose(6);
  quad_pose = pose2Vec(pose);

  this->body = this->model->GetChildLink("quad::chasis");
  this->quadrotor = QuadrotorModel(quad_pose);

  // gazebo node
  // clang-format off
  GazeboNode::configure();
  this->registerPublisher<POSE_MSG>(POSE_GTOPIC);
  this->registerPublisher<VELOCITY_MSG>(VELOCITY_GTOPIC);
  this->registerSubscriber(ATT_SETPOINT_GTOPIC, &QuadrotorGPlugin::setAttitudeCallback, this);
  this->registerSubscriber(POS_SETPOINT_GTOPIC, &QuadrotorGPlugin::setPositionCallback, this);
  this->registerSubscriber(VEL_SETPOINT_GTOPIC, &QuadrotorGPlugin::setVelocityCallback, this);
  // clang-format on
}

void QuadrotorGPlugin::onUpdate(const gazebo::common::UpdateInfo &info) {
  gazebo::common::Time diff = info.simTime - this->prev_sim_time;
  double dt = diff.nsec / 1000000000.0;  // convert nsec to sec
  this->simulate(dt);
  this->prev_sim_time = info.simTime;
}

void QuadrotorGPlugin::simulate(double dt) {
  // pre-check
  if (dt < 0.0) {
    return;
  }

  // set quadrotor orientation using last calculated keep the following code
  // block above the `this->quadrotor.update()` had the 3D model been set
  // using
  // the calculated position in the Quadrotor class it would go through
  // objects
  // and cause collision detection to go wild
  VecX quad_pose = this->quadrotor.getPose();
  ignition::math::Pose3d gazebo_pose = this->model->WorldPose();
  gazebo_pose = ignition::math::Pose3d(gazebo_pose.Pos().X(),
                                       gazebo_pose.Pos().Y(),
                                       gazebo_pose.Pos().Z(),
                                       quad_pose(3),
                                       quad_pose(4),
                                       quad_pose(5));
  this->model->SetWorldPose(gazebo_pose);
  this->pose = gazebo_pose;

  // simulate quadrotor
  // TODO: switch to attitude controller when in offboard mode
  Vec4 motor_inputs = this->quadrotor.attitudeControllerControl(dt);
  // motor_inputs = this->quadrotor.positionControllerControl(dt);
  // motor_inputs = this->quadrotor.velocityControllerControl(dt);
  this->quadrotor.update(motor_inputs, dt);

  // set model pose
  quad_pose = this->quadrotor.getPose();
  gazebo_pose = vec2pose(quad_pose);
  this->model->SetWorldPose(gazebo_pose);

  // publish
  this->publishPose();
  this->publishVelocity();
}

void QuadrotorGPlugin::publishPose() {
  gazebo::msgs::Pose msg;

  msg.mutable_position()->set_x(this->pose.Pos().X());
  msg.mutable_position()->set_y(this->pose.Pos().Y());
  msg.mutable_position()->set_z(this->pose.Pos().Z());
  msg.mutable_orientation()->set_w(this->pose.Rot().W());
  msg.mutable_orientation()->set_x(this->pose.Rot().X());
  msg.mutable_orientation()->set_y(this->pose.Rot().Y());
  msg.mutable_orientation()->set_z(this->pose.Rot().Z());

  this->gaz_pubs[POSE_GTOPIC]->Publish(msg);
}

void QuadrotorGPlugin::publishVelocity() {
  gazebo::msgs::Vector3d msg;

  msg.set_x(this->quadrotor.linear_velocity(0));
  msg.set_y(this->quadrotor.linear_velocity(1));
  msg.set_z(this->quadrotor.linear_velocity(2));

  this->gaz_pubs[VELOCITY_GTOPIC]->Publish(msg);
}

void QuadrotorGPlugin::setAttitudeCallback(AttitudeSetpointPtr &msg) {
  // clang-format off
  this->quadrotor.setAttitude(msg->roll(),
                              msg->pitch(),
                              msg->yaw(),
                              msg->throttle());
  // clang-format on
}

void QuadrotorGPlugin::setPositionCallback(PositionSetpointPtr &msg) {
  this->quadrotor.setPosition(msg->x(), msg->y(), msg->z());
}

void QuadrotorGPlugin::setVelocityCallback(VelocitySetpointPtr &msg) {
  this->quadrotor.setVelocity(msg->vx(), msg->vy(), msg->vz());
}

GZ_REGISTER_MODEL_PLUGIN(QuadrotorGPlugin);
}  // namespace gaz
}  // namespace atl
