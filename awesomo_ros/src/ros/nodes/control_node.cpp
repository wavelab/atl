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
  this->registerSubscriber(QMODE_TOPIC, &ControlNode::modeCallback, this);
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

void ControlNode::modeCallback(const std_msgs::String &msg) {
  std::string mode;

  convertMsg(msg, mode);
  if (mode == "DISARM_MODE") {
    this->quadrotor.setMode(DISARM_MODE);
  } else if (mode == "HOVER_MODE") {
    this->quadrotor.setMode(HOVER_MODE);
  } else if (mode == "DISCOVER_MODE") {
    this->quadrotor.setMode(DISCOVER_MODE);
  } else if (mode == "TRACKING_MODE") {
    this->quadrotor.setMode(TRACKING_MODE);
  } else if (mode == "LANDING_MODE") {
    this->quadrotor.setMode(LANDING_MODE);
  }
}

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
  this->mavros_state = *msg;
}

void ControlNode::poseCallback(const geometry_msgs::PoseStamped &msg) {
  Pose pose;
  convertMsg(msg, pose);
  this->quadrotor.setPose(pose);
}

void ControlNode::velocityCallback(const geometry_msgs::TwistStamped &msg) {
  Vec3 linear_velocity;
  convertMsg(msg.twist.linear, linear_velocity);
  this->quadrotor.setVelocity(linear_velocity);
}

void ControlNode::radioCallback(const mavros_msgs::RCIn &msg) {
  for (int i = 0; i < 16; i++) {
    this->rc_in[i] = msg.channels[i];
  }
  if (this->armed) {
      if (this->rc_in[6] < 1500) {
          this->armed = false;
          this->disarm();
      }
  } else {
      if (this->rc_in[6] > 1500) {
          this->armed = true;
          this->setOffboardModeOn();
      }
  }

}

void ControlNode::targetPositionCallback(const geometry_msgs::Vector3 &msg) {
  Vec3 position;
  convertMsg(msg, position);
  this->quadrotor.setTargetPosition(position);
}

void ControlNode::targetVelocityCallback(const geometry_msgs::Vector3 &msg) {
  Vec3 velocity;
  convertMsg(msg, velocity);
  this->quadrotor.setTargetVelocity(velocity);
}

void ControlNode::targetDetectedCallback(const std_msgs::Bool &msg) {
  this->quadrotor.setTargetDetected(msg.data);
}

void ControlNode::headingSetCallback(const std_msgs::Float64 &msg) {
  convertMsg(msg, this->quadrotor.heading);
}

void ControlNode::hoverSetCallback(const geometry_msgs::Vector3 &msg) {
  convertMsg(msg, this->quadrotor.hover_position);
}

void ControlNode::hoverHeightSetCallback(const std_msgs::Float64 &msg) {
  convertMsg(msg, this->quadrotor.hover_position(2));
}

void ControlNode::positionControllerSetCallback(
  const awesomo_msgs::PCtrlSettings &msg) {
  convertMsg(msg, this->quadrotor.position_controller);
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

  if (this->armed) {
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
  }

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
