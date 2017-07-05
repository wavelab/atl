#include "atl_gazebo/bridge/dji_quadrotor_node.hpp"


namespace atl {
namespace gazebo_bridge {

int DJIQuadrotorNode::configure(const std::string &node_name, int hz) {
  // setup ros node
  // clang-format off
  ROSNode::configure(node_name, hz);
  ROSNode::registerPublisher<dji_sdk::LocalPosition>(DJI_LOCAL_POSITION_RTOPIC);
  ROSNode::registerPublisher<dji_sdk::AttitudeQuaternion>(DJI_ATTITUDE_QUATERNION_RTOPIC);
  ROSNode::registerPublisher<dji_sdk::Velocity>(DJI_VELOCITY_RTOPIC);
  ROSNode::registerServer(DJI_ATTITUDE_SETPOINT_RTOPIC, &DJIQuadrotorNode::attitudeControlCallback, this);
  // clang-format on

  // setup gazebo client
  if (QuadrotorGClient::configure() != 0) {
    ROS_ERROR("Failed to configure QuadrotorGClient!");
    return -1;
  }

  return 0;
}

void DJIQuadrotorNode::poseCallback(RPYPosePtr &msg) {
  std::random_device rd;
  std::mt19937 x_gen(rd());
  std::mt19937 y_gen(rd());
  std::mt19937 z_gen(rd());
  std::normal_distribution<> x_err(0, 0.0);
  std::normal_distribution<> y_err(0, 0.0);
  std::normal_distribution<> z_err(0, 0.0);
  dji_sdk::LocalPosition local_position_msg;
  dji_sdk::AttitudeQuaternion attitude_msg;
  Vec3 ros_pos, ros_euler;
  Quaternion ros_quat;

  // gazebo pose callback
  QuadrotorGClient::poseCallback(msg);

  // transform from NWU to NED
  ros_pos(0) = this->pose(0) + x_err(x_gen);
  ros_pos(1) = -this->pose(1) + y_err(y_gen);
  ros_pos(2) = -this->pose(2) + z_err(z_gen);

  // convert euler angles from NWU to NED and then to quaternion
  ros_euler << msg->roll(), -msg->pitch(), -msg->yaw();
  euler2quat(ros_euler, 321, ros_quat);

  // build local position and attitude msg
  local_position_msg.x = ros_pos(0);
  local_position_msg.y = ros_pos(1);
  local_position_msg.z = ros_pos(2);

  attitude_msg.q0 = ros_quat.w();
  attitude_msg.q1 = ros_quat.x();
  attitude_msg.q2 = ros_quat.y();
  attitude_msg.q3 = ros_quat.z();

  // publish
  this->ros_pubs[DJI_LOCAL_POSITION_RTOPIC].publish(local_position_msg);
  this->ros_pubs[DJI_ATTITUDE_QUATERNION_RTOPIC].publish(attitude_msg);
}

void DJIQuadrotorNode::velocityCallback(ConstVector3dPtr &msg) {
  // gazebo velocity callback
  QuadrotorGClient::velocityCallback(msg);

  // build ros msg
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> err(0, 0.0);
  dji_sdk::Velocity velocity_msg;

  // transform velocity from NWU to NED
  velocity_msg.vx = this->velocity(0) + err(gen);
  velocity_msg.vy = -this->velocity(1) + err(gen);
  velocity_msg.vz = -this->velocity(2) + err(gen);

  this->ros_pubs[DJI_VELOCITY_RTOPIC].publish(velocity_msg);
}

bool DJIQuadrotorNode::attitudeControlCallback(
  dji_sdk::AttitudeControl::Request &request,
  dji_sdk::AttitudeControl::Response &response) {
  Vec3 euler;

  // pre-check
  if (request.flag != 0x20) {
    log_err("Attitude control byte other than [0x20] is not supported!");
    response.result = false;
    return false;
  }

  // transform roll pitch yaw from NED to NWU
  euler << deg2rad(request.x), -deg2rad(request.y),
    wrapTo180(-deg2rad(request.yaw));

  // set attitude
  this->setAttitude(euler(0), euler(1), euler(2), request.z / 100.0);

  // return
  response.result = true;
  return true;
}

}  // namespace gazebo_bridge
}  // namespace atl

RUN_ROS_NODE(atl::gazebo_bridge::DJIQuadrotorNode, NODE_NAME, NODE_RATE);
