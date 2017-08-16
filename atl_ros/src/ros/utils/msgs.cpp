#include "atl/ros/utils/msgs.hpp"

namespace atl {

void buildMsg(bool b, std_msgs::Bool &msg) { msg.data = b; }

void buildMsg(std::string s, std_msgs::String &msg) { msg.data = s; }

void buildMsg(double d, std_msgs::Float64 &msg) { msg.data = d; }

void buildMsg(Vec3 vec, geometry_msgs::Vector3 &msg) {
  msg.x = vec(0);
  msg.y = vec(1);
  msg.z = vec(2);
}

void buildMsg(Vec3 vec, geometry_msgs::Point &msg) {
  msg.x = vec(0);
  msg.y = vec(1);
  msg.z = vec(2);
}

void buildMsg(Quaternion q, geometry_msgs::Quaternion &msg) {
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
}

void buildMsg(int seq,
              ros::Time time,
              Pose pose,
              geometry_msgs::PoseStamped &msg) {
  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = "atl_quadrotor_pose";

  msg.pose.position.x = pose.position(0);
  msg.pose.position.y = pose.position(1);
  msg.pose.position.z = pose.position(2);

  msg.pose.orientation.w = pose.orientation.w();
  msg.pose.orientation.x = pose.orientation.x();
  msg.pose.orientation.y = pose.orientation.y();
  msg.pose.orientation.z = pose.orientation.z();
}

void buildMsg(TagPose tag, atl_msgs::AprilTagPose &msg) {
  msg.id = tag.id;
  msg.detected = tag.detected;

  msg.position.x = tag.position(0);
  msg.position.y = tag.position(1);
  msg.position.z = tag.position(2);

  msg.orientation.w = tag.orientation.w();
  msg.orientation.x = tag.orientation.x();
  msg.orientation.y = tag.orientation.y();
  msg.orientation.z = tag.orientation.z();
}

void buildMsg(TagPose tag, geometry_msgs::Vector3 &msg) {
  msg.x = tag.position(0);
  msg.y = tag.position(1);
  msg.z = tag.position(2);
}

void buildMsg(PositionController pc, atl_msgs::PCtrlSettings &msg) {
  msg.roll_controller.min = pc.roll_limit[0];
  msg.roll_controller.max = pc.roll_limit[1];
  msg.roll_controller.k_p = pc.x_controller.k_p;
  msg.roll_controller.k_i = pc.x_controller.k_i;
  msg.roll_controller.k_d = pc.x_controller.k_d;

  msg.pitch_controller.min = pc.pitch_limit[0];
  msg.pitch_controller.max = pc.pitch_limit[1];
  msg.pitch_controller.k_p = pc.y_controller.k_p;
  msg.pitch_controller.k_i = pc.y_controller.k_i;
  msg.pitch_controller.k_d = pc.y_controller.k_d;

  msg.throttle_controller.k_p = pc.z_controller.k_p;
  msg.throttle_controller.k_i = pc.z_controller.k_i;
  msg.throttle_controller.k_d = pc.z_controller.k_d;
  msg.hover_throttle = pc.hover_throttle;
}

void buildMsg(TrackingController tc, atl_msgs::TCtrlSettings &msg) {
  msg.roll_controller.k_p = tc.x_controller.k_p;
  msg.roll_controller.k_i = tc.x_controller.k_i;
  msg.roll_controller.k_d = tc.x_controller.k_d;

  msg.pitch_controller.k_p = tc.y_controller.k_p;
  msg.pitch_controller.k_i = tc.y_controller.k_i;
  msg.pitch_controller.k_d = tc.y_controller.k_d;

  msg.throttle_controller.k_p = tc.z_controller.k_p;
  msg.throttle_controller.k_i = tc.z_controller.k_i;
  msg.throttle_controller.k_d = tc.z_controller.k_d;
  msg.hover_throttle = tc.hover_throttle;

  msg.roll_controller.min = tc.roll_limit[0];
  msg.roll_controller.max = tc.roll_limit[1];

  msg.pitch_controller.min = tc.pitch_limit[0];
  msg.pitch_controller.max = tc.pitch_limit[1];

  msg.track_offset.x = tc.track_offset(0);
  msg.track_offset.y = tc.track_offset(1);
  msg.track_offset.z = tc.track_offset(2);
}

// void buildMsg(LandingController lc, atl_msgs::LCtrlSettings &msg) {
//   msg.vx_controller.k_p = lc.vx_controller.k_p;
//   msg.vx_controller.k_i = lc.vx_controller.k_i;
//   msg.vx_controller.k_d = lc.vx_controller.k_d;
//
//   msg.vy_controller.k_p = lc.vy_controller.k_p;
//   msg.vy_controller.k_i = lc.vy_controller.k_i;
//   msg.vy_controller.k_d = lc.vy_controller.k_d;
//
//   msg.vz_controller.k_p = lc.vz_controller.k_p;
//   msg.vz_controller.k_i = lc.vz_controller.k_i;
//   msg.vz_controller.k_d = lc.vz_controller.k_d;
//
//   msg.roll_controller.min = lc.roll_limit[0];
//   msg.roll_controller.max = lc.roll_limit[1];
//
//   msg.pitch_controller.min = lc.pitch_limit[0];
//   msg.pitch_controller.max = lc.pitch_limit[1];
//
//   msg.throttle_controller.min = lc.throttle_limit[0];
//   msg.throttle_controller.max = lc.throttle_limit[1];
// }

void convertMsg(std_msgs::Bool msg, bool &b) { b = msg.data; }

void convertMsg(std_msgs::String msg, std::string &s) { s = msg.data; }

void convertMsg(std_msgs::Float64 msg, double &d) { d = msg.data; }

void convertMsg(geometry_msgs::Vector3 msg, Vec3 &v) {
  v << msg.x, msg.y, msg.z;
}

void convertMsg(geometry_msgs::Vector3Stamped msg, Vec3 &v) {
  v << msg.vector.x, msg.vector.y, msg.vector.z;
}

void convertMsg(geometry_msgs::Point msg, Vec3 &v) { v << msg.x, msg.y, msg.z; }

void convertMsg(geometry_msgs::Quaternion msg, Quaternion &q) {
  q.w() = msg.w;
  q.x() = msg.x;
  q.y() = msg.y;
  q.z() = msg.z;
}

void convertMsg(geometry_msgs::QuaternionStamped msg, Quaternion &q) {
  q.w() = msg.quaternion.w;
  q.x() = msg.quaternion.x;
  q.y() = msg.quaternion.y;
  q.z() = msg.quaternion.z;
}

void convertMsg(geometry_msgs::PoseStamped msg, Pose &p) {
  convertMsg(msg.pose.position, p.position);
  convertMsg(msg.pose.orientation, p.orientation);
}

void convertMsg(geometry_msgs::TwistStamped msg, VecX &v) {
  v(0) = msg.twist.linear.x;
  v(1) = msg.twist.linear.y;
  v(2) = msg.twist.linear.z;

  v(3) = msg.twist.angular.x;
  v(4) = msg.twist.angular.y;
  v(5) = msg.twist.angular.z;
}

void convertMsg(atl_msgs::AprilTagPose msg, TagPose &tag) {
  tag.id = msg.id;
  tag.detected = msg.detected;
  convertMsg(msg.position, tag.position);
  convertMsg(msg.orientation, tag.orientation);
}

void convertMsg(atl_msgs::PCtrlSettings msg, PositionController &pc) {
  pc.pitch_limit[0] = deg2rad(msg.pitch_controller.min);
  pc.pitch_limit[1] = deg2rad(msg.pitch_controller.max);
  pc.x_controller.k_p = msg.pitch_controller.k_p;
  pc.x_controller.k_i = msg.pitch_controller.k_i;
  pc.x_controller.k_d = msg.pitch_controller.k_d;

  pc.roll_limit[0] = deg2rad(msg.roll_controller.min);
  pc.roll_limit[1] = deg2rad(msg.roll_controller.max);
  pc.y_controller.k_p = msg.roll_controller.k_p;
  pc.y_controller.k_i = msg.roll_controller.k_i;
  pc.y_controller.k_d = msg.roll_controller.k_d;

  pc.z_controller.k_p = msg.throttle_controller.k_p;
  pc.z_controller.k_i = msg.throttle_controller.k_i;
  pc.z_controller.k_d = msg.throttle_controller.k_d;
  pc.hover_throttle = msg.hover_throttle;
}

void convertMsg(atl_msgs::TCtrlSettings msg, TrackingController &tc) {
  tc.y_controller.k_p = msg.roll_controller.k_p;
  tc.y_controller.k_i = msg.roll_controller.k_i;
  tc.y_controller.k_d = msg.roll_controller.k_d;

  tc.x_controller.k_p = msg.pitch_controller.k_p;
  tc.x_controller.k_i = msg.pitch_controller.k_i;
  tc.x_controller.k_d = msg.pitch_controller.k_d;

  tc.z_controller.k_p = msg.throttle_controller.k_p;
  tc.z_controller.k_i = msg.throttle_controller.k_i;
  tc.z_controller.k_d = msg.throttle_controller.k_d;
  tc.hover_throttle = msg.hover_throttle;

  tc.roll_limit[0] = deg2rad(msg.roll_controller.min);
  tc.roll_limit[1] = deg2rad(msg.roll_controller.max);

  tc.pitch_limit[0] = deg2rad(msg.pitch_controller.min);
  tc.pitch_limit[1] = deg2rad(msg.pitch_controller.max);

  // clang-format off
  tc.track_offset << msg.track_offset.x,
                     msg.track_offset.y,
                     msg.track_offset.z;
  // clang-format on
}

// void convertMsg(atl_msgs::LCtrlSettings msg, LandingController &lc) {
//   lc.vx_controller.k_p = msg.vx_controller.k_p;
//   lc.vx_controller.k_i = msg.vx_controller.k_i;
//   lc.vx_controller.k_d = msg.vx_controller.k_d;
//
//   lc.vy_controller.k_p = msg.vy_controller.k_p;
//   lc.vy_controller.k_i = msg.vy_controller.k_i;
//   lc.vy_controller.k_d = msg.vy_controller.k_d;
//
//   lc.vz_controller.k_p = msg.vz_controller.k_p;
//   lc.vz_controller.k_i = msg.vz_controller.k_i;
//   lc.vz_controller.k_d = msg.vz_controller.k_d;
//
//   lc.roll_limit[0] = deg2rad(msg.roll_controller.min);
//   lc.roll_limit[1] = deg2rad(msg.roll_controller.max);
//
//   lc.pitch_limit[0] = deg2rad(msg.pitch_controller.min);
//   lc.pitch_limit[1] = deg2rad(msg.pitch_controller.max);
//
//   lc.throttle_limit[0] = msg.throttle_controller.min;
//   lc.throttle_limit[1] = msg.throttle_controller.max;
// }

} // namespace atl
