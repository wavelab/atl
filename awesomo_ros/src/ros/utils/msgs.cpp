#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

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

  msg.pose.orientation.w = att_cmd.q.w();
  msg.pose.orientation.x = att_cmd.q.x();
  msg.pose.orientation.y = att_cmd.q.y();
  msg.pose.orientation.z = att_cmd.q.z();

  // throttle command
  thr_msg.data = att_cmd.throttle;
}

void buildMsg(TagPose tag, awesomo_msgs::AprilTagPose &msg) {
  msg.tag_id = tag.id;
  msg.tag_detected = tag.detected;
  msg.tag_position.x = tag.position(0);
  msg.tag_position.y = tag.position(1);
  msg.tag_position.z = tag.position(2);
}

void buildMsg(TagPose tag, geometry_msgs::Vector3 &msg) {
  msg.x = tag.position(0);
  msg.y = tag.position(1);
  msg.z = tag.position(2);
}

void buildMsg(PositionController controller, awesomo_msgs::PCtrlStats &msg) {
  // roll
  msg.roll_p_error = controller.y_controller.error_p;
  msg.roll_i_error = controller.y_controller.error_i;
  msg.roll_d_error = controller.y_controller.error_d;
  msg.roll_output = rad2deg(controller.outputs(0));
  msg.roll_setpoint = controller.setpoints(0);

  // pitch
  msg.pitch_p_error = controller.x_controller.error_p;
  msg.pitch_i_error = controller.x_controller.error_i;
  msg.pitch_d_error = controller.x_controller.error_d;
  msg.pitch_output = rad2deg(controller.outputs(1));
  msg.pitch_setpoint = controller.setpoints(1);

  // thrust
  msg.throttle_p_error = controller.z_controller.error_p;
  msg.throttle_i_error = controller.z_controller.error_i;
  msg.throttle_d_error = controller.z_controller.error_d;
  msg.throttle_output = controller.outputs(3);
  msg.throttle_setpoint = controller.setpoints(2);
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

void buildMsg(PositionController controller, awesomo_msgs::TCtrlStats &msg) {
  // roll
  msg.roll_p_error = controller.y_controller.error_p;
  msg.roll_i_error = controller.y_controller.error_i;
  msg.roll_d_error = controller.y_controller.error_d;
  msg.roll_output = rad2deg(controller.outputs(0));
  msg.roll_setpoint = controller.setpoints(0);

  // pitch
  msg.pitch_p_error = controller.x_controller.error_p;
  msg.pitch_i_error = controller.x_controller.error_i;
  msg.pitch_d_error = controller.x_controller.error_d;
  msg.pitch_output = rad2deg(controller.outputs(1));
  msg.pitch_setpoint = controller.setpoints(1);

  // thrust
  msg.throttle_p_error = controller.z_controller.error_p;
  msg.throttle_i_error = controller.z_controller.error_i;
  msg.throttle_d_error = controller.z_controller.error_d;
  msg.throttle_output = controller.outputs(3);
  msg.throttle_setpoint = controller.setpoints(2);
}

void buildMsg(Vec3 vec, geometry_msgs::Vector3 &msg) {
  msg.x = vec(0);
  msg.y = vec(1);
  msg.z = vec(2);
}

Pose convertMsg(geometry_msgs::PoseStamped msg) {
  Vec3 p;
  Quaternion q;
  geometry_msgs::Quaternion orientation;

  orientation = msg.pose.orientation;
  q = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
  p << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

  return Pose(q, p);
}

VecX convertMsg(geometry_msgs::TwistStamped msg) {
  VecX twist(6);

  twist(0) = msg.twist.linear.x;
  twist(1) = msg.twist.linear.y;
  twist(2) = msg.twist.linear.z;

  twist(3) = msg.twist.angular.x;
  twist(4) = msg.twist.angular.y;
  twist(5) = msg.twist.angular.z;

  return twist;
}

Vec3 convertMsg(geometry_msgs::Vector3 msg) {
  Vec3 v;
  v << msg.x, msg.y, msg.z;
  return v;
}

TagPose convertMsg(awesomo_msgs::AprilTagPose msg) {
  int id;
  bool detected;
  Vec3 p;

  id = msg.tag_id;
  detected = msg.tag_detected;
  p << msg.tag_position.x, msg.tag_position.y, msg.tag_position.z;

  return TagPose(id, detected, p);
}

}  // end of awesomo namespace
