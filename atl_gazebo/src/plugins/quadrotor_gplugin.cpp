#include "wavesim_gazebo/plugins/quadrotor_gplugin.hpp"


namespace wavesim {
namespace gaz {

VecX pose2Vec(gazebo::math::Pose pose) {
  gazebo::math::Vector3 pos;
  gazebo::math::Vector3 rpy;
  VecX eigen_pose(6);

  pos = pose.pos;
  rpy = pose.rot.GetAsEuler();
  eigen_pose << pos.x, pos.y, pos.z, rpy.x, rpy.y, rpy.z;

  return eigen_pose;
}

gazebo::math::Pose vec2pose(VecX pose) {
  float x, y, z;
  float phi, theta, psi;
  gazebo::math::Pose gazebo_pose;

  x = pose(0);
  y = pose(1);
  z = pose(2);
  phi = pose(3);
  theta = pose(4);
  psi = pose(5);
  gazebo_pose = gazebo::math::Pose(x, y, z, phi, theta, psi);

  return gazebo_pose;
}

QuadrotorGPlugin::QuadrotorGPlugin(void) {
  printf("LOADING [libquadrotor_gplugin.so]!\n");
}

void QuadrotorGPlugin::Load(gazebo::physics::ModelPtr model,
                            sdf::ElementPtr sdf) {
  gazebo::math::Pose pose;
  VecX quad_pose(6);

  // set model and bind world update callback
  // clang-format off
  this->model = model;
  this->update_conn = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&QuadrotorGPlugin::onUpdate, this, _1)
  );
  // clang-format on

  // set quadrotor model initial pose
  this->model->SetGravityMode(false);
  pose = this->model->GetWorldPose();
  quad_pose = pose2Vec(pose);
  this->body = this->model->GetChildLink("quad::chasis");
  this->quadrotor = wave::quadrotor::QuadrotorModel(quad_pose);

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
  double dt;
  gazebo::common::Time diff;

  diff = info.simTime - this->prev_sim_time;
  dt = diff.nsec / 1000000000.0;  // convert nsec to sec
  this->simulate(dt);
  this->prev_sim_time = info.simTime;
}

void QuadrotorGPlugin::simulate(double dt) {
  VecX motor_inputs(4);
  VecX quad_pose;
  gazebo::math::Pose gazebo_pose;

  // pre-check
  if (dt < 0.0) {
    return;
  }

  // set quadrotor orientation using last calculated
  // keep the following code block above the `this->quadrotor.update()` had
  // the
  // 3D model been set using the calculated position in the Quadrotor class it
  // would go through objects and cause collision detection to go wild
  quad_pose = this->quadrotor.getPose();
  gazebo_pose = this->model->GetWorldPose();
  gazebo_pose = gazebo::math::Pose(gazebo_pose.pos.x,
                                   gazebo_pose.pos.y,
                                   gazebo_pose.pos.z,
                                   quad_pose(3),
                                   quad_pose(4),
                                   quad_pose(5));
  this->model->SetWorldPose(gazebo_pose);

  // simulate quadrotor
  // TODO: switch to attitude controller when in offboard mode
  motor_inputs = this->quadrotor.attitudeControllerControl(dt);
  // motor_inputs = this->quadrotor.positionControllerControl(dt);
  // motor_inputs = this->quadrotor.velocityControllerControl(dt);
  this->quadrotor.update(motor_inputs, dt);

  // set model pose
  quad_pose = this->quadrotor.getPose();
  gazebo_pose = vec2pose(quad_pose);
  this->model->SetWorldPose(gazebo_pose);

  // set model linear velocities
  // gazebo::math::Vector3 vector;
  // vector.x = this->quadrotor.states(9);
  // vector.y = this->quadrotor.states(10);
  // vector.z = this->quadrotor.states(11);
  // this->body->SetLinearVel(vector);

  // THIS IS A HACK TO PREVENT QUAD FROM VIBRATING AT REST
  // if (motor_inputs.norm() > 0.5) {
  //   this->body->SetLinearVel(vector);
  //
  // } else {
  //   vector.x = 0;
  //   vector.y = 0;
  //   vector.z = 0;
  //   this->body->SetForce(vector);
  //   this->body->SetLinearVel(vector);
  //   this->body->SetLinearAccel(vector);
  // }

  // publish
  this->publishPose();
  this->publishVelocity();
}

void QuadrotorGPlugin::publishPose(void) {
  wavesim_msgs::msgs::RPYPose msg;

  msg.set_x(this->quadrotor.position(0));
  msg.set_y(this->quadrotor.position(1));
  msg.set_z(this->quadrotor.position(2));
  msg.set_roll(this->quadrotor.attitude(0));
  msg.set_pitch(this->quadrotor.attitude(1));
  msg.set_yaw(this->quadrotor.attitude(2));

  this->gaz_pubs[POSE_GTOPIC]->Publish(msg);
}

void QuadrotorGPlugin::publishVelocity(void) {
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
}  // end of gaz namespace
}  // end of wavesim namespace
