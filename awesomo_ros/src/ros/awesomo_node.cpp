#include "awesomo_ros/awesomo_node.hpp"


namespace awesomo {

AwesomoNode::AwesomoNode(void) {
  this->configured = false;

  for (int i = 0; i < 16; i++) {
    this->rc_in[i] = 0.0f;
  }
}

int AwesomoNode::configure(const std::string node_name, int hz) {
  std::string config_path;

  // ros node
  ROSNode::configure(node_name, hz);

  // quadrotor
  this->ros_nh->getParam("/config_dir", config_path);
  if (this->quadrotor.configure(config_path) != 0) {
    log_err(FCONFQUAD);
    return -1;
  }

  // services
  // clang-format off
  this->mode_client = this->ros_nh->serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
  this->arming_client = this->ros_nh->serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);
  // clang-format on

  // subscribers
  // clang-format off
  ROSNode::registerSubscriber(STATE_TOPIC, &AwesomoNode::stateCallback, this);
  ROSNode::registerSubscriber(POSE_TOPIC, &AwesomoNode::poseCallback, this);
  ROSNode::registerSubscriber(RADIO_TOPIC, &AwesomoNode::radioCallback, this);
  ROSNode::registerSubscriber(APRILTAG_TOPIC, &AwesomoNode::aprilTagCallback, this);
  // clang-format on

  // publishers
  // clang-format off
  ROSNode::registerPublisher<geometry_msgs::PoseStamped>(SETPOINT_ATTITUDE_TOPIC);
  ROSNode::registerPublisher<std_msgs::Float64>(SETPOINT_THROTTLE_TOPIC);
  ROSNode::registerPublisher<geometry_msgs::PoseStamped>(SETPOINT_POSITION_TOPIC);
  ROSNode::registerPublisher<awesomo_msgs::PositionControllerStats>(POS_CONTROLLER_STATS_TOPIC);
  ROSNode::registerPublisher<awesomo_msgs::KFStats>(KF_STATS_TOPIC);
  ROSNode::registerPublisher<awesomo_msgs::KFPlotting>(KF_PLOTTING_TOPIC);
  // clang-format on

  // loop callback
  // clang-format off
  ROSNode::registerLoopCallback(std::bind(&AwesomoNode::awesomoLoopCallback, this));
  // clang-format on

  // wait till connected to FCU
  this->waitForConnection();

  this->configured = true;
  return 0;
}

void AwesomoNode::waitForConnection(void) {
  bool sim_mode;

  // pre-check
  sim_mode = false;
  this->ros_nh->getParam("/sim_mode", sim_mode);
  if (sim_mode == true) {
    ROS_INFO("Running simulation mode!");
    return;
  }

  // wait fo FCU
  ROS_INFO("Waiting for FCU ...");
  while (this->mavros_state.connected != true) {
    ros::spinOnce();
    sleep(1);
  }
  ROS_INFO("Connected to FCU!");
}

int AwesomoNode::disarm(void) {
  mavros_msgs::CommandBool msg;

  // setup
  ROS_INFO("Disarming awesomo ...");
  msg.request.value = false;

  // disarm
  if (this->arming_client.call(msg)) {
    ROS_INFO("Awesomo disarmed!");
    return 0;
  } else {
    ROS_ERROR("Failed to disarm awesomo!");
    return -1;
  }
}

int AwesomoNode::setOffboardModeOn(void) {
  mavros_msgs::SetMode msg;

  // setup
  msg.request.custom_mode = "OFFBOARD";

  if (this->mode_client.call(msg) && msg.response.success) {
    ROS_INFO("OFFBOARD MODE ON!");
    return 0;
  } else {
    ROS_ERROR("Failed to enable offboard mode!");
    return -1;
  }
}

void AwesomoNode::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
  this->mavros_state = *msg;
}

void AwesomoNode::poseCallback(const geometry_msgs::PoseStamped &msg) {
  this->world_pose = convertPoseStampedMsg2Pose(msg);
}

void AwesomoNode::radioCallback(const mavros_msgs::RCIn &msg) {
  for (int i = 0; i < 16; i++) {
    this->rc_in[i] = msg.channels[i];
  }
}

void AwesomoNode::aprilTagCallback(const awesomo_msgs::AprilTagPose &msg) {
  this->tag_pose = convertAprilTagPoseMsg2TagPose(msg);
}

int AwesomoNode::awesomoLoopCallback(void) {
  int seq;
  double dt;
  AttitudeCommand att_cmd;
  std_msgs::Float64 thr_msg;
  geometry_msgs::PoseStamped att_msg;

  // setup
  seq = this->ros_seq;
  dt = (ros::Time::now() - this->ros_last_updated).toSec();

  // step
  if (this->quadrotor.step(this->world_pose, dt) != 0) {
    return -1;
  }

  // publish msgs
  att_cmd = this->quadrotor.att_cmd;
  buildAttitudeMsg(seq, ros::Time::now(), att_cmd, att_msg, thr_msg);
  this->ros_pubs[SETPOINT_ATTITUDE_TOPIC].publish(att_msg);
  this->ros_pubs[SETPOINT_THROTTLE_TOPIC].publish(thr_msg);

  return 0;
}

void AwesomoNode::publishStats(void) {
  int seq;
  ros::Time time;
  awesomo_msgs::PositionControllerStats pcs_stats_msg;
  awesomo_msgs::KFStats kf_stats_msg;
  awesomo_msgs::KFPlotting kf_plot_msg;

  // setup
  seq = this->ros_seq;
  time = ros::Time::now();

  // build msgs
  // clang-format off
  buildPositionControllerStatsMsg(seq, time, this->quadrotor.position_controller, pcs_stats_msg);
  // buildKFStatsMsg(seq, time, this->quadrotor.apriltag_estimator, kf_stats_msg);
  // buildKFPlottingMsg(seq, time, this->quadrotor.apriltag_estimator, kf_plot_msg);
  // clang-format on

  // publish
  this->ros_pubs[POS_CONTROLLER_STATS_TOPIC].publish(pcs_stats_msg);
  // this->ros_pubs[KF_STATS_TOPIC].publish(kf_stats_msg);
  // this->ros_pubs[KF_PLOTTING_TOPIC].publish(kf_plot_msg);
}

}  // end of awesomo namespace

int main(int argc, char **argv) {
  awesomo::AwesomoNode awesomo_node;

  if (awesomo_node.configure(NODE_NAME, NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure AwesomoNode!");
    return -1;
  }
  awesomo_node.loop();

  return 0;
}
