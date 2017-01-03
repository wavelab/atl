#include "awesomo_ros/control_node.hpp"


namespace awesomo {

ControlNode::ControlNode(void) {
  for (int i = 0; i < 16; i++) {
    this->rc_in[i] = 0.0f;
  }
}

int ControlNode::configure(const std::string node_name, int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // quadrotor
  this->ros_nh->getParam("/control_config_dir", config_path);
  if (this->quadrotor.configure(config_path) != 0) {
    ROS_ERROR(FCONFQUAD);
    return -2;
  }

  // services
  // clang-format off
  this->mode_client = this->ros_nh->serviceClient<mavros_msgs::SetMode>("mode_topic");
  this->arming_client = this->ros_nh->serviceClient<mavros_msgs::CommandBool>("arm_topic");
  // clang-format on

  // subscribers
  // clang-format off
  ROSNode::registerSubscriber("state_topic", &ControlNode::stateCallback, this);
  ROSNode::registerSubscriber("pose_topic", &ControlNode::poseCallback, this);
  ROSNode::registerSubscriber("radio_topic", &ControlNode::radioCallback, this);
  ROSNode::registerSubscriber("apriltag_topic", &ControlNode::aprilTagCallback, this);
  // clang-format on

  // publishers
  // clang-format off
  ROSNode::registerPublisher<geometry_msgs::PoseStamped>("setpoint_attitude_topic");
  ROSNode::registerPublisher<std_msgs::Float64>("setpoint_throttle_topic");
  ROSNode::registerPublisher<geometry_msgs::PoseStamped>("setpoint_position_topic");
  ROSNode::registerPublisher<awesomo_msgs::PositionControllerStats>("controller_stats_topic");
  ROSNode::registerPublisher<awesomo_msgs::KFStats>("kf_stats_topic");
  ROSNode::registerPublisher<awesomo_msgs::KFPlotting>("kf_plotting_topic");
  // clang-format on

  // loop callback
  // clang-format off
  ROSNode::registerLoopCallback(std::bind(&ControlNode::awesomoLoopCallback, this));
  // clang-format on

  // wait till connected to FCU
  this->waitForConnection();

  this->configured = true;
  return 0;
}

void ControlNode::waitForConnection(void) {
  // pre-check
  if (this->sim_mode) {
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

int ControlNode::disarm(void) {
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

int ControlNode::setOffboardModeOn(void) {
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

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
  this->mavros_state = *msg;
}

void ControlNode::poseCallback(const geometry_msgs::PoseStamped &msg) {
  this->world_pose = convertPoseStampedMsg2Pose(msg);
}

void ControlNode::radioCallback(const mavros_msgs::RCIn &msg) {
  for (int i = 0; i < 16; i++) {
    this->rc_in[i] = msg.channels[i];
  }
}

void ControlNode::aprilTagCallback(const awesomo_msgs::AprilTagPose &msg) {
  this->tag_pose = convertAprilTagPoseMsg2TagPose(msg);
}

int ControlNode::awesomoLoopCallback(void) {
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
  this->ros_pubs["setpoint_attitude_topic"].publish(att_msg);
  this->ros_pubs["setpoint_throttle_topic"].publish(thr_msg);

  return 0;
}

void ControlNode::publishStats(void) {
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
  this->ros_pubs["pos_controller_stats_topic"].publish(pcs_stats_msg);
  // this->ros_pubs["kf_stats_topic"].publish(kf_stats_msg);
  // this->ros_pubs["kf_plotting_topic"].publish(kf_plot_msg);
}

}  // end of awesomo namespace

int main(int argc, char **argv) {
  awesomo::ControlNode awesomo_node;

  if (awesomo_node.configure(CONTROL_NODE_NAME, CONTROL_NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure ControlNode!");
    return -1;
  }
  awesomo_node.loop();

  return 0;
}
