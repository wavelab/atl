#ifndef __AWESOMO_ROS_MSGS_HPP__
#define __AWESOMO_ROS_MSGS_HPP__

#include <geometry_msgs/Quaternion.h>

#include <awesomo_core/awesomo_core.hpp>

#include <awesomo_msgs/KFPlotting.h>
#include <awesomo_msgs/KFStats.h>
#include <awesomo_msgs/PositionControllerStats.h>


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
void buildPositionControllerStatsMsg(int seq,
                                     ros::Time time,
                                     PositionController controller,
                                     awesomo_msgs::PositionControllerStats &msg) {
  // msg header
  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = "awesomo_pos_controller_stats";

  // roll
  msg.roll_p_error = controller.x_controller.error_p;
  msg.roll_i_error = controller.x_controller.error_i;
  msg.roll_d_error = controller.x_controller.error_d;
  msg.roll_output = controller.output_roll * 180 / M_PI;
  msg.roll_setpoint = controller.setpoint_x;

  // pitch
  msg.pitch_p_error = controller.y_controller.error_p;
  msg.pitch_i_error = controller.y_controller.error_i;
  msg.pitch_d_error = controller.y_controller.error_d;
  msg.pitch_output = controller.output_pitch * 180 / M_PI;
  msg.pitch_setpoint = controller.setpoint_y;

  // thrust
  msg.throttle_p_error = controller.z_controller.error_p;
  msg.throttle_i_error = controller.z_controller.error_i;
  msg.throttle_d_error = controller.z_controller.error_d;
  msg.throttle_output = controller.output_throttle;
  msg.throttle_setpoint = controller.setpoint_z;
}
// clang-format on

void buildPositionControllerMsg(int seq,
                                ros::Time time,
                                PositionController controller,
                                geometry_msgs::PoseStamped &msg,
                                std_msgs::Float64 throttle) {
  // atitude command
  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = "awesomo_pos_controller_cmd";

  msg.pose.position.x = 0;
  msg.pose.position.y = 0;
  msg.pose.position.z = 0;

  // msg.pose.orientation.x = controller->command_quat.x();
  // msg.pose.orientation.y = controller->command_quat.y();
  // msg.pose.orientation.z = controller->command_quat.z();
  // msg.pose.orientation.w = controller->command_quat.w();

  // throttle command
  throttle.data = controller.output_throttle;
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
void buildKFPlottingMsg(int seq,
                        ros::Time time,
                        KalmanFilter estimator,
                        awesomo_msgs::KFPlotting &msg) {
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
#endif
