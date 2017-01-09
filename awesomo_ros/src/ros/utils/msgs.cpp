#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// #define MAT2MSG(X, Y)                    \
//   msg.Y_rows = X.rows();                 \
//   msg.Y_cols = X.cols();                 \
//   for (int i = 0; i < X.rows(); i++) {   \
//     for (int j = 0; j < X.cols(); j++) { \
//       msg.Y_data[i] = X(i, j);           \
//     }                                    \
//   }
//
// #define VEC2MSG(X, Y)                  \
//   msg.Y_size = X.size();               \
//   for (int i = 0; i < X.size(); i++) { \
//     msg.Y_data[i] = X(i);              \
//   }

// clang-format off
void buildPCtrlStatsMsg(int seq,
                        ros::Time time,
                        PositionController controller,
                        awesomo_msgs::PCtrlStats &msg) {
  // msg header
  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = "awesomo_pos_controller_stats";

  // roll
  msg.roll_p_error = controller.y_controller.error_p;
  msg.roll_i_error = controller.y_controller.error_i;
  msg.roll_d_error = controller.y_controller.error_d;
  msg.roll_output = controller.outputs(0) * 180 / M_PI;
  msg.roll_setpoint = controller.setpoints(0);

  // pitch
  msg.pitch_p_error = controller.x_controller.error_p;
  msg.pitch_i_error = controller.x_controller.error_i;
  msg.pitch_d_error = controller.x_controller.error_d;
  msg.pitch_output = controller.outputs(1) * 180 / M_PI;
  msg.pitch_setpoint = controller.setpoints(1);

  // thrust
  msg.throttle_p_error = controller.z_controller.error_p;
  msg.throttle_i_error = controller.z_controller.error_i;
  msg.throttle_d_error = controller.z_controller.error_d;
  msg.throttle_output = controller.outputs(3);
  msg.throttle_setpoint = controller.setpoints(2);
}
// clang-format on

void buildAttitudeMsg(int seq,
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

void buildKFStatsMsg(int seq,
                     ros::Time time,
                     KalmanFilter estimator,
                     awesomo_msgs::KFStats &msg) {
  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = "awesomo_kf_stats";

  // MAT2MSG(estimator.A, A);
  // MAT2MSG(estimator.B, B);
  // MAT2MSG(estimator.R, R);
  // MAT2MSG(estimator.C, C);
  // MAT2MSG(estimator.Q, Q);
  // MAT2MSG(estimator.S, S);
  // MAT2MSG(estimator.K, K);
  // VEC2MSG(estimator.mu, mu);
  // VEC2MSG(estimator.mu_p, mu_p);
  // VEC2MSG(estimator.S_p, S_p);
}

// clang-format off
void buildKFPlotMsg(int seq,
                        ros::Time time,
                        KalmanFilter estimator,
                        awesomo_msgs::KFPlot &msg) {
  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = "awesomo_kf_plotting";

  msg.x = estimator.mu(0);
  msg.y = estimator.mu(1);
  msg.z = estimator.mu(2);

  msg.vel_x = estimator.mu(3);
  msg.vel_y = estimator.mu(4);
  msg.vel_z = estimator.mu(5);

  msg.acc_x = estimator.mu(6);
  msg.acc_y = estimator.mu(7);
  msg.acc_z = estimator.mu(8);
}

void buildAprilTagPoseMsg(TagPose tag, awesomo_msgs::AprilTagPose &msg) {
  msg.tag_id = tag.id;
  msg.tag_detected = tag.detected;
  msg.tag_position.x = tag.position(0);
  msg.tag_position.y = tag.position(1);
  msg.tag_position.z = tag.position(2);
}

void buildAprilTagTrackMsg(TagPose tag, geometry_msgs::Vector3 &msg) {
  msg.x = tag.position(0);
  msg.y = tag.position(1);
  msg.z = tag.position(2);
}

void buildPCtrlSettingsMsg(PositionController pc,
                           awesomo_msgs::PCtrlSettings &msg) {
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

Pose convertPoseStampedMsg2Pose(geometry_msgs::PoseStamped msg) {
  Vec3 p;
  Quaternion q;
  geometry_msgs::Quaternion orientation;

  orientation = msg.pose.orientation;
  q = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
  p << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

  return Pose(q, p);
}

TagPose convertAprilTagPoseMsg2TagPose(awesomo_msgs::AprilTagPose msg) {
  int id;
  bool detected;
  Vec3 p;

  id = msg.tag_id;
  detected = msg.tag_detected;
  p << msg.tag_position.x, msg.tag_position.y, msg.tag_position.z;

  return TagPose(id, detected, p);
}

}  // end of awesomo namespace
