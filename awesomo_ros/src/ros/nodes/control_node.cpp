#include "awesomo_ros/nodes/control_node.hpp"


namespace awesomo {

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
  this->mode_client = this->ros_nh->serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
  this->arming_client = this->ros_nh->serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);
  // clang-format on

  // publishers
  // clang-format off
  this->registerPublisher<geometry_msgs::PoseStamped>(SETPOINT_ATTITUDE_TOPIC, 1);
  this->registerPublisher<std_msgs::Float64>(SETPOINT_THROTTLE_TOPIC, 1);
  this->registerPublisher<geometry_msgs::PoseStamped>(SETPOINT_POSITION_TOPIC);
  this->registerPublisher<awesomo_msgs::PCtrlStats>(PCTRL_STATS_TOPIC);
  this->registerPublisher<awesomo_msgs::PCtrlSettings>(PCTRL_GET_TOPIC);
  // clang-format on

  // subscribers
  // clang-format off
  this->registerSubscriber(STATE_TOPIC, &ControlNode::stateCallback, this);
  this->registerSubscriber(POSE_TOPIC, &ControlNode::poseCallback, this);
  this->registerSubscriber(VELOCITY_TOPIC, &ControlNode::velocityCallback, this);
  this->registerSubscriber(RADIO_TOPIC, &ControlNode::radioCallback, this);
  this->registerSubscriber(TARGET_BODY_POSITION_TOPIC, &ControlNode::targetPositionCallback, this);
  this->registerSubscriber(TARGET_BODY_VELOCITY_TOPIC, &ControlNode::targetVelocityCallback, this);
  this->registerSubscriber(TARGET_DETECTED_TOPIC, &ControlNode::targetDetectedCallback, this);
  this->registerSubscriber(HEADING_SET_TOPIC, &ControlNode::headingSetCallback, this);
  this->registerSubscriber(HOVER_SET_TOPIC, &ControlNode::hoverSetCallback, this);
  this->registerSubscriber(HOVER_HEIGHT_SET_TOPIC, &ControlNode::hoverHeightSetCallback, this);
  this->registerSubscriber(PCTRL_SET_TOPIC, &ControlNode::positionControllerSetCallback, this);
  // clang-format on

  // loop callback
  // clang-format off
  this->registerLoopCallback(std::bind(&ControlNode::loopCallback, this));
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
  Pose world_pose = convertMsg(msg);
  this->quadrotor.setPose(world_pose);
}

void ControlNode::velocityCallback(const geometry_msgs::TwistStamped &msg) {
  Vec3 linear_velocity = convertMsg(msg.twist.linear);
  this->quadrotor.setVelocity(linear_velocity);
}

void ControlNode::radioCallback(const mavros_msgs::RCIn &msg) {
  for (int i = 0; i < 16; i++) {
    this->rc_in[i] = msg.channels[i];
  }
}

void ControlNode::targetPositionCallback(const geometry_msgs::Vector3 &msg) {
  Vec3 position = convertMsg(msg);
  this->quadrotor.setTargetPosition(position);
}

void ControlNode::targetVelocityCallback(const geometry_msgs::Vector3 &msg) {
  Vec3 velocity = convertMsg(msg);
  this->quadrotor.setTargetVelocity(velocity);
}

void ControlNode::targetDetectedCallback(const std_msgs::Bool &msg) {
  this->quadrotor.setTargetDetected(msg.data);
}

void ControlNode::headingSetCallback(const std_msgs::Float64 &msg) {
  this->quadrotor.heading = convertMsg(msg);
}

void ControlNode::hoverSetCallback(const geometry_msgs::Vector3 &msg) {
  this->quadrotor.hover_position = convertMsg(msg);
}

void ControlNode::hoverHeightSetCallback(const std_msgs::Float64 &msg) {
  this->quadrotor.hover_position(2) = msg.data;
}

void ControlNode::positionControllerSetCallback(
  const awesomo_msgs::PCtrlSettings &msg) {
  PositionController *position_controller;

  position_controller = &this->quadrotor.position_controller;

  position_controller->pitch_limit[0] = deg2rad(msg.pitch_controller.min);
  position_controller->pitch_limit[1] = deg2rad(msg.pitch_controller.max);
  position_controller->x_controller.k_p = msg.pitch_controller.k_p;
  position_controller->x_controller.k_i = msg.pitch_controller.k_i;
  position_controller->x_controller.k_d = msg.pitch_controller.k_d;

  position_controller->roll_limit[0] = deg2rad(msg.roll_controller.min);
  position_controller->roll_limit[1] = deg2rad(msg.roll_controller.max);
  position_controller->y_controller.k_p = msg.roll_controller.k_p;
  position_controller->y_controller.k_i = msg.roll_controller.k_i;
  position_controller->y_controller.k_d = msg.roll_controller.k_d;

  position_controller->z_controller.k_p = msg.throttle_controller.k_p;
  position_controller->z_controller.k_i = msg.throttle_controller.k_i;
  position_controller->z_controller.k_d = msg.throttle_controller.k_d;
  position_controller->hover_throttle = msg.hover_throttle;
}

int ControlNode::loopCallback(void) {
  int seq;
  double dt;
  AttitudeCommand att_cmd;
  std_msgs::Float64 thr_msg;
  geometry_msgs::PoseStamped att_msg;

  // setup
  seq = this->ros_seq;
  dt = (ros::Time::now() - this->ros_last_updated).toSec();

  // step
  if (this->quadrotor.step(dt) != 0) {
    return -1;
  }

  // publish msgs
  att_cmd = this->quadrotor.att_cmd;
  buildMsg(seq, ros::Time::now(), att_cmd, att_msg, thr_msg);
  this->ros_pubs[SETPOINT_ATTITUDE_TOPIC].publish(att_msg);
  this->ros_pubs[SETPOINT_THROTTLE_TOPIC].publish(thr_msg);
  this->publishStats();

  return 0;
}

void ControlNode::publishStats(void) {
  int seq;
  ros::Time time;
  awesomo_msgs::PCtrlStats pc_stats_msg;
  awesomo_msgs::TCtrlStats tc_stats_msg;
  awesomo_msgs::VCtrlStats vc_stats_msg;

  // setup
  seq = this->ros_seq;
  time = ros::Time::now();

  // build msgs
  // clang-format off
  buildMsg(this->quadrotor.position_controller, pc_stats_msg);
  // clang-format on

  // publish
  // this->ros_pubs[PCTRL_STATS_TOPIC].publish(pcs_stats_msg);
  // this->ros_pubs[PCTRL_GET].publish(pcs_settings_msg);
}

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::ControlNode, NODE_NAME, NODE_RATE);
