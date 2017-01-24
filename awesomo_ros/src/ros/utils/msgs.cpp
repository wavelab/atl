#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

void buildMsg(bool b, std_msgs::Bool &msg) {
  msg.data = b;
}

void buildMsg(std::string s, std_msgs::String &msg) {
  msg.data = s;
}

void buildMsg(double d, std_msgs::Float64 &msg) {
  msg.data = d;
}

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
              AttitudeCommand att_cmd,
              geometry_msgs::PoseStamped &msg,
              std_msgs::Float64 &thr_msg) {
  // atitude command
  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = "awesomo_attitude_cmd";

  msg.pose.position.x = 0;
  msg.pose.position.y = 0;
  msg.pose.position.z = 0;

  msg.pose.orientation.w = att_cmd.orientation.w();
  msg.pose.orientation.x = att_cmd.orientation.x();
  msg.pose.orientation.y = att_cmd.orientation.y();
  msg.pose.orientation.z = att_cmd.orientation.z();

  // throttle command
  thr_msg.data = att_cmd.throttle;
}

void buildMsg(TagPose tag, awesomo_msgs::AprilTagPose &msg) {
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

void buildMsg(PositionController pc, awesomo_msgs::PCtrlStats &msg) {
  // roll
  msg.roll_p_error = pc.y_controller.error_p;
  msg.roll_i_error = pc.y_controller.error_i;
  msg.roll_d_error = pc.y_controller.error_d;
  msg.roll_output = rad2deg(pc.outputs(0));
  msg.roll_setpoint = pc.setpoints(0);

  // pitch
  msg.pitch_p_error = pc.x_controller.error_p;
  msg.pitch_i_error = pc.x_controller.error_i;
  msg.pitch_d_error = pc.x_controller.error_d;
  msg.pitch_output = rad2deg(pc.outputs(1));
  msg.pitch_setpoint = pc.setpoints(1);

  // thrust
  msg.throttle_p_error = pc.z_controller.error_p;
  msg.throttle_i_error = pc.z_controller.error_i;
  msg.throttle_d_error = pc.z_controller.error_d;
  msg.throttle_output = pc.outputs(3);
  msg.throttle_setpoint = pc.setpoints(2);
}

void buildMsg(PositionController pc, awesomo_msgs::PCtrlSettings &msg) {
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

void buildMsg(TrackingController tc, awesomo_msgs::TCtrlStats &msg) {
  // roll
  msg.roll_p_error = tc.y_controller.error_p;
  msg.roll_i_error = tc.y_controller.error_i;
  msg.roll_d_error = tc.y_controller.error_d;
  msg.roll_output = rad2deg(tc.outputs(0));
  msg.roll_setpoint = tc.setpoints(0);

  // pitch
  msg.pitch_p_error = tc.x_controller.error_p;
  msg.pitch_i_error = tc.x_controller.error_i;
  msg.pitch_d_error = tc.x_controller.error_d;
  msg.pitch_output = rad2deg(tc.outputs(1));
  msg.pitch_setpoint = tc.setpoints(1);

  // thrust
  msg.throttle_p_error = tc.z_controller.error_p;
  msg.throttle_i_error = tc.z_controller.error_i;
  msg.throttle_d_error = tc.z_controller.error_d;
  msg.throttle_output = tc.outputs(3);
  msg.throttle_setpoint = tc.setpoints(2);
}

void buildMsg(TrackingController tc, awesomo_msgs::TCtrlSettings &msg) {
  msg.roll_controller.min = tc.roll_limit[0];
  msg.roll_controller.max = tc.roll_limit[1];
  msg.roll_controller.k_p = tc.x_controller.k_p;
  msg.roll_controller.k_i = tc.x_controller.k_i;
  msg.roll_controller.k_d = tc.x_controller.k_d;

  msg.pitch_controller.min = tc.pitch_limit[0];
  msg.pitch_controller.max = tc.pitch_limit[1];
  msg.pitch_controller.k_p = tc.y_controller.k_p;
  msg.pitch_controller.k_i = tc.y_controller.k_i;
  msg.pitch_controller.k_d = tc.y_controller.k_d;

  msg.throttle_controller.k_p = tc.z_controller.k_p;
  msg.throttle_controller.k_i = tc.z_controller.k_i;
  msg.throttle_controller.k_d = tc.z_controller.k_d;
  msg.hover_throttle = tc.hover_throttle;
}

int convertMsg(std_msgs::Bool msg, bool &b) {
  b = msg.data;
  return 0;
}

int convertMsg(std_msgs::String msg, std::string &s) {
  s = msg.data;
  return 0;
}

int convertMsg(std_msgs::Float64 msg, double &d) {
  d = msg.data;
  return 0;
}

int convertMsg(geometry_msgs::Vector3 msg, Vec3 &v) {
  v << msg.x, msg.y, msg.z;
  return 0;
}

int convertMsg(geometry_msgs::Point msg, Vec3 &v) {
  v << msg.x, msg.y, msg.z;
  return 0;
}

int convertMsg(geometry_msgs::Quaternion msg, Quaternion &q) {
  q.w() = msg.w;
  q.x() = msg.x;
  q.y() = msg.y;
  q.z() = msg.z;
  return 0;
}

int convertMsg(geometry_msgs::PoseStamped msg, Pose &p) {
  convertMsg(msg.pose.orientation, p.q);
  convertMsg(msg.pose.position, p.position);
  return 0;
}

int convertMsg(geometry_msgs::TwistStamped msg, VecX &v) {
  v(0) = msg.twist.linear.x;
  v(1) = msg.twist.linear.y;
  v(2) = msg.twist.linear.z;

  v(3) = msg.twist.angular.x;
  v(4) = msg.twist.angular.y;
  v(5) = msg.twist.angular.z;

  return 0;
}

int convertMsg(awesomo_msgs::AprilTagPose msg, TagPose &tag) {
  tag.id = msg.id;
  tag.detected = msg.detected;
  convertMsg(msg.position, tag.position);
  convertMsg(msg.orientation, tag.orientation);

  return 0;
}

int convertMsg(awesomo_msgs::PCtrlSettings msg, PositionController &pc) {
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

  return 0;
}

}  // end of awesomo namespace
