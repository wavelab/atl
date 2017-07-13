#include "atl/gazebo/bridge/dji_quadrotor_node.hpp"

namespace atl {
namespace gazebo_bridge {

int DJIQuadrotorNode::configure(const std::string &node_name, int hz) {
  // setup ros node
  // clang-format off
  ROSNode::configure(node_name, hz);
  ROSNode::registerPublisher<geometry_msgs::NatSatFix>(DJI_GPS_POSITION_RTOPIC);
  ROSNode::registerPublisher<geometry_msgs::QuaternionStamped>(DJI_ATTITUDE_RTOPIC);
  ROSNode::registerPublisher<geometry_msgs::Vector3Stamped>(DJI_VELOCITY_RTOPIC);
  ROSNode::registerSubscriber(DJI_SETPOINT_RTOPIC, &DJIQuadrotorNode::controlCallback, this);
  // clang-format on

  // setup gazebo client
  if (QuadrotorGClient::configure() != 0) {
    ROS_ERROR("Failed to configure QuadrotorGClient!");
    return -1;
  }

  return 0;
}

void DJIQuadrotorNode::poseGazeboCallback(RPYPosePtr &msg) {
  std::random_device rd;
  std::mt19937 x_gen(rd());
  std::mt19937 y_gen(rd());
  std::mt19937 z_gen(rd());
  std::normal_distribution<> x_err(0, 0.0);
  std::normal_distribution<> y_err(0, 0.0);
  std::normal_distribution<> z_err(0, 0.0);

  // gazebo pose callback
  QuadrotorGClient::poseCallback(msg);

  // convert euler angles from NWU to ENU and then to quaternion
  Vec3 ros_euler;
  ros_euler(0) = -msg->pitch();
  ros_euler(1) = msg->roll();
  ros_euler(2) = msg->yaw();
  Quaternion ros_quat;
  euler2quat(ros_euler, 321, ros_quat);

  // convert NWU position to GPS lat lon coordinates
  double lat = 0.0;
  double lon = 0.0;
  latlon_offset(this->home_latitude,
                this->home_longitude,
                this->pose(0) + x_err(x_gen),
                this->pose(1) + y_err(y_gen),
                &lat,
                &lon);

  // build gps msg
  geometry_msgs::NatSatFix gps_msg;
  gps_msg.latitude = lat;
  gps_msg.longitude = lon;
  gps_msg.altitude = this->pose(2) + z_err(z_gen);

  // build attitude msg
  geometry_msgs::QuaternionStamped att_msg;
  att_msg.q0 = ros_quat.w();
  att_msg.q1 = ros_quat.x();
  att_msg.q2 = ros_quat.y();
  att_msg.q3 = ros_quat.z();

  // publish
  this->ros_pubs[DJI_GPS_POSITION_RTOPIC].publish(gps_msg);
  this->ros_pubs[DJI_ATTITUDE_RTOPIC].publish(att_msg);
}

void DJIQuadrotorNode::velocityGazeboCallback(ConstVector3dPtr &msg) {
  // gazebo velocity callback
  QuadrotorGClient::velocityCallback(msg);

  // build ros msg
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> err(0, 0.0);

  // transform velocity from NWU to ENU
  geometry_msgs::Vector3Stamped velocity_msg;
  velocity_msg.vector.x = -this->velocity(1) + err(gen);
  velocity_msg.vector.y = this->velocity(0) + err(gen);
  velocity_msg.vector.z = this->velocity(2) + err(gen);

  this->ros_pubs[DJI_VELOCITY_RTOPIC].publish(velocity_msg);
}

void DJIQuadrotorNode::attitudeSetpointCallback(const sensor_msgs::Joy &msg) {
  double roll = msg.axes[0];
  double pitch = msg.axes[1];
  double yaw = msg.axes[2];
  double throttle = msg.axes[3];
  int control_flag = msg.axes[4];

  // pre-check
  if (control_flag != 0x20) {
    LOG_ERROR("Attitude control byte other than [0x20] is not supported!");
    return;
  }

  // transform roll pitch yaw from NED to NWU
  Vec3 euler{deg2rad(roll), -deg2rad(pitch), wrapTo180(-deg2rad(yaw))};

  // set attitude
  this->setAttitude(euler(0), euler(1), euler(2), throttle / 100.0);
}

}  // namespace gazebo_bridge
}  // namespace atl

RUN_ROS_NODE(atl::gazebo_bridge::DJIQuadrotorNode, NODE_NAME, NODE_RATE);
