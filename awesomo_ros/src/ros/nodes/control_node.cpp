#include "awesomo_ros/nodes/control_node.hpp"


namespace awesomo {

int ControlNode::configure(const std::string node_name, int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // quadrotor
  ROS_GET_PARAM("/control_config_dir", config_path);
  ROS_GET_PARAM("/quad_frame", this->quad_frame);
  ROS_GET_PARAM("/fcu_type", this->fcu_type);
  if (this->quadrotor.configure(config_path) != 0) {
    ROS_ERROR(FCONFQUAD);
    return -2;
  }

  // configure PX4 or DJI topics
  if (this->fcu_type == "PX4") {
    this->configurePX4Topics();

  } else if (this->fcu_type == "DJI") {
    this->dji = new DJIDrone(*this->ros_nh);
    this->configureDJITopics();

  } else {
    ROS_ERROR("Invalid [fcu_type]: %s", this->fcu_type.c_str());
    return -1;
  }

  // publishers
  this->registerPublisher<awesomo_msgs::PCtrlStats>(PCTRL_STATS_TOPIC);
  this->registerPublisher<awesomo_msgs::PCtrlSettings>(PCTRL_GET_TOPIC);
  this->registerPublisher<geometry_msgs::PoseStamped>(QUADROTOR_POSE);
  this->registerPublisher<geometry_msgs::TwistStamped>(QUADROTOR_VELOCITY);
  this->registerPublisher<std_msgs::Bool>(ESTIMATOR_ON_TOPIC);
  this->registerPublisher<std_msgs::Bool>(ESTIMATOR_OFF_TOPIC);

  // subscribers
  // clang-format off
  this->registerSubscriber(MODE_TOPIC, &ControlNode::modeCallback, this);
  this->registerSubscriber(HEADING_TOPIC, &ControlNode::headingCallback, this);
  this->registerSubscriber(TARGET_BODY_POSITION_TOPIC, &ControlNode::targetPositionCallback, this);
  this->registerSubscriber(TARGET_BODY_VELOCITY_TOPIC, &ControlNode::targetVelocityCallback, this);
  this->registerSubscriber(TARGET_DETECTED_TOPIC, &ControlNode::targetDetectedCallback, this);
  this->registerSubscriber(HOVER_SET_TOPIC, &ControlNode::hoverSetCallback, this);
  this->registerSubscriber(HOVER_HEIGHT_SET_TOPIC, &ControlNode::hoverHeightSetCallback, this);
  this->registerSubscriber(PCTRL_SET_TOPIC, &ControlNode::positionControllerSetCallback, this);
  this->registerSubscriber(TCTRL_SET_TOPIC, &ControlNode::trackingControllerSetCallback, this);
  this->registerSubscriber(LCTRL_SET_TOPIC, &ControlNode::landingControllerSetCallback, this);
  // clang-format on

  // loop callback
  this->registerLoopCallback(std::bind(&ControlNode::loopCallback, this));

  // connect to FCU
  if (this->fcu_type == "PX4" && this->px4Connect() != 0) {
    return -2;
  }

  // connect to estimator
  this->waitForEstimator();

  this->configured = true;
  return 0;
}

int ControlNode::configurePX4Topics(void) {
  // clang-format off
  // services
  this->px4_mode_client = this->ros_nh->serviceClient<mavros_msgs::SetMode>(PX4_MODE_TOPIC);
  this->px4_arming_client = this->ros_nh->serviceClient<mavros_msgs::CommandBool>(PX4_ARM_TOPIC);

  // publishers
  this->registerPublisher<geometry_msgs::PoseStamped>(PX4_SETPOINT_ATTITUDE_TOPIC);
  this->registerPublisher<std_msgs::Float64>(PX4_SETPOINT_THROTTLE_TOPIC);
  this->registerPublisher<geometry_msgs::PoseStamped>(PX4_SETPOINT_POSITION_TOPIC);

  // subscribers
  this->registerSubscriber(PX4_STATE_TOPIC, &ControlNode::px4StateCallback, this);
  this->registerSubscriber(PX4_POSE_TOPIC, &ControlNode::px4PoseCallback, this);
  this->registerSubscriber(PX4_VELOCITY_TOPIC, &ControlNode::px4VelocityCallback, this);
  this->registerSubscriber(PX4_RADIO_TOPIC, &ControlNode::px4RadioCallback, this);
  // clang-format on

  return 0;
}

int ControlNode::configureDJITopics(void) {
  // subscribers
  // clang-format off
  this->registerSubscriber(DJI_POSITION_TOPIC, &ControlNode::djiPositionCallback, this);
  this->registerSubscriber(DJI_ATTITUDE_TOPIC, &ControlNode::djiAttitudeCallback, this);
  this->registerSubscriber(DJI_VELOCITY_TOPIC, &ControlNode::djiVelocityCallback, this);
  this->registerSubscriber(DJI_RADIO_TOPIC, &ControlNode::djiRadioCallback, this);
  // clang-format on

  return 0;
}

int ControlNode::px4Connect(void) {
  int attempts;

  // pre-check
  if (this->sim_mode) {
    return 0;
  }

  // wait fo FCU
  attempts = 0;
  log_info("Waiting for PX4 FCU ...");
  while (this->px4_state.connected != true) {
    if (attempts == 10) {
      log_info("Failed to connect to PX4 FCU for 10 seconds ...");
      return -1;
    }

    sleep(1);
    ros::spinOnce();
    attempts++;
  }

  log_info("Connected to PX4 FCU!");
  return 0;
}

int ControlNode::px4Disarm(void) {
  mavros_msgs::CommandBool msg;

  // setup
  log_info("Disarming awesomo ...");
  msg.request.value = false;

  // disarm
  if (this->px4_arming_client.call(msg)) {
    log_info("PX4 FCU disarmed!");
    return 0;
  } else {
    log_err("Failed to disarm PX4 FCU!");
    return -1;
  }
}

int ControlNode::px4OffboardModeOn(void) {
  mavros_msgs::SetMode msg;
  msg.request.custom_mode = "OFFBOARD";

  if (this->px4_mode_client.call(msg) && msg.response.success) {
    log_info("PX4 FCU OFFBOARD MODE ON!");
    return 0;
  } else {
    log_err("Failed to enable PX4 FCU offboard mode!");
    return -1;
  }
}

int ControlNode::djiDisarm(void) {
  if (this->dji->drone_disarm() != true) {
    return -1;
  }

  return 0;
}

int ControlNode::djiOffboardModeOn(void) {
  if (this->dji->request_sdk_permission_control() != true) {
    log_err("Failed to release DJI SDK control!");
    return -1;
  }
  log_info("Obtained DJI SDK control!");

  return 0;
}

int ControlNode::djiOffboardModeOff(void) {
  if (this->dji->release_sdk_permission_control() != true) {
    log_err("Failed to release DJI SDK control!");
    return -1;
  }
  log_info("Released DJI SDK control!");

  return 0;
}

int ControlNode::waitForEstimator(void) {
  int attempts;

  // wait for estimator
  log_info("Waiting for Estimator ...");
  attempts = 0;

  while (this->ros_pubs[ESTIMATOR_ON_TOPIC].getNumSubscribers() == 0) {
    if (attempts == 10) {
      log_info("Failed to connect to EstimatorNode for 10 seconds ...");
      return -1;
    }

    ros::spinOnce();
    sleep(1);
    attempts++;
  }

  // switch estimator on
  this->setEstimatorOn();

  return 0;
}

void ControlNode::setEstimatorOn(void) {
  std_msgs::Bool msg;
  msg.data = true;
  this->ros_pubs[ESTIMATOR_ON_TOPIC].publish(msg);
}

void ControlNode::setEstimatorOff(void) {
  std_msgs::Bool msg;
  msg.data = true;
  this->ros_pubs[ESTIMATOR_OFF_TOPIC].publish(msg);
}

void ControlNode::px4StateCallback(const mavros_msgs::State::ConstPtr &msg) {
  this->px4_state = *msg;
}

void ControlNode::px4PoseCallback(const geometry_msgs::PoseStamped &msg) {
  Pose pose;
  Vec3 position;
  Quaternion orientation;

  // convert message to pose
  convertMsg(msg, pose);
  if (this->quad_frame == "NWU") {
    nwu2enu(pose.position, position);
    pose.position = position;

  } else if (this->quad_frame == "NED") {
    ned2enu(pose.position, position);
    ned2nwu(pose.orientation, orientation);
    pose.position = position;
    pose.orientation = orientation;

  } else {
    log_err("Invalid ROS [/quad_frame] param value: %s", this->quad_frame.c_str());

  }

  this->quadrotor.setPose(pose);
}

void ControlNode::px4VelocityCallback(const geometry_msgs::TwistStamped &msg) {
  Vec3 vel_nwu, vel_ned, vel_enu;

  if (this->quad_frame == "NWU") {
    convertMsg(msg.twist.linear, vel_nwu);
    nwu2enu(vel_nwu, vel_enu);
    this->quadrotor.setVelocity(vel_enu);

  } else if (this->quad_frame == "NED") {
    convertMsg(msg.twist.linear, vel_ned);
    ned2enu(vel_ned, vel_enu);
    this->quadrotor.setVelocity(vel_enu);

  } else {
    log_err("Invalid ROS [/quad_frame] param value: %s", this->quad_frame.c_str());

  }
}

void ControlNode::px4RadioCallback(const mavros_msgs::RCIn &msg) {
  for (int i = 0; i < 16; i++) {
    this->rc_in[i] = msg.channels[i];
  }

  if (this->armed) {
    if (this->rc_in[6] < 1500) {
      this->armed = false;
      this->quadrotor.setMode(HOVER_MODE);
    }
  } else {
    if (this->rc_in[6] > 1500) {
      this->armed = true;
      this->quadrotor.setMode(DISCOVER_MODE);
    }
  }
}

void ControlNode::djiPositionCallback(const dji_sdk::LocalPosition &msg) {
  Vec3 pos_ned, pos_enu;

  pos_ned(0) = msg.x;
  pos_ned(1) = msg.y;
  pos_ned(2) = msg.z;
  ned2enu(pos_ned, pos_enu);

  this->quadrotor.pose.position = pos_enu;
}

void ControlNode::djiAttitudeCallback(const dji_sdk::AttitudeQuaternion &msg) {
  Quaternion orientation_ned, orientation_nwu;

  orientation_ned.w() = msg.q0;
  orientation_ned.x() = msg.q1;
  orientation_ned.y() = msg.q2;
  orientation_ned.z() = msg.q3;

  // transform pose position and orientation
  // from NED to ENU and NWU
  ned2nwu(orientation_ned, orientation_nwu);

  this->quadrotor.pose.orientation = orientation_nwu;
}

void ControlNode::djiVelocityCallback(const dji_sdk::Velocity &msg) {
  Vec3 vel_ned, vel_enu;

  // convert DJI msg to Eigen vector
  vel_ned(0) = msg.vx;
  vel_ned(1) = msg.vy;
  vel_ned(2) = msg.vz;

  // transform velocity in NED to ENU
  ned2enu(vel_ned, vel_enu);

  // update
  this->quadrotor.setVelocity(vel_enu);
}

void ControlNode::djiRadioCallback(const dji_sdk::RCChannels &msg) {
  if (this->armed) {
    if (msg.gear < 0) {
      this->armed = false;
      this->djiOffboardModeOff();
    }
  } else {
    if (msg.gear > 0) {
      this->armed = true;
      this->quadrotor.setMode(DISCOVER_MODE);
      this->djiOffboardModeOn();
    }
  }
}

void ControlNode::modeCallback(const std_msgs::String &msg) {
  std::string mode;

  // setup
  convertMsg(msg, mode);

  // parse mode
  if (mode == "DISARM_MODE") {
    this->setEstimatorOff();
    this->quadrotor.setMode(DISARM_MODE);
  } else if (mode == "HOVER_MODE") {
    this->setEstimatorOff();
    this->quadrotor.setMode(HOVER_MODE);
  } else if (mode == "DISCOVER_MODE") {
    this->setEstimatorOn();
    this->quadrotor.setMode(DISCOVER_MODE);
  } else if (mode == "TRACKING_MODE") {
    this->setEstimatorOn();
    this->quadrotor.setMode(TRACKING_MODE);
  } else if (mode == "LANDING_MODE") {
    this->setEstimatorOn();
    this->quadrotor.setMode(LANDING_MODE);
  }
}

void ControlNode::headingCallback(const std_msgs::Float64 &msg) {
  double heading;
  convertMsg(msg, heading);
  this->quadrotor.setHeading(heading);
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

void ControlNode::trackingControllerSetCallback(
  const awesomo_msgs::TCtrlSettings &msg) {
  convertMsg(msg, this->quadrotor.tracking_controller);
}

void ControlNode::landingControllerSetCallback(
  const awesomo_msgs::LCtrlSettings &msg) {
  convertMsg(msg, this->quadrotor.landing_controller);
}

void ControlNode::publishAttitudeSetpoint(void) {
  int seq;
  Vec3 euler;
  Quaternion q_ned;
  AttitudeCommand att_cmd;
  std_msgs::Float64 thr_msg;
  geometry_msgs::PoseStamped att_msg;

  // setup
  seq = this->ros_seq;
  att_cmd = this->quadrotor.att_cmd;

  if (this->fcu_type == "PX4") {
    buildMsg(seq, ros::Time::now(), att_cmd, att_msg, thr_msg);
    this->ros_pubs[PX4_SETPOINT_ATTITUDE_TOPIC].publish(att_msg);
    this->ros_pubs[PX4_SETPOINT_THROTTLE_TOPIC].publish(thr_msg);

  } else if (this->fcu_type == "DJI") {
    // transform orientation from NWU to NED
    nwu2ned(att_cmd.orientation, q_ned);
    quat2euler(q_ned, 321, euler);

    //  DJI Control Mode Byte
    //
    //    bit 7:6  0b00: HORI_ATTI_TILT_ANG
    //             0b01: HORI_VEL
    //             0b10: HORI_POS
    //
    //    bit 5:4  0b00: VERT_VEL
    //             0b01: VERT_POS
    //             0b10: VERT_THRUST
    //
    //    bit 3    0b0: YAW_ANG
    //             0b1: YAW_RATE
    //
    //    bit 2:1  0b00: horizontal frame is ground frame
    //             0b01: horizontal frame is body frame
    //
    //    bit 0    0b0: non-stable mode
    //             0b1: stable mode
    //
    //  We used:
    //
    //    HORIZ_ATTI_TILT_ANG
    //    VERT_THRUST
    //    YAW_ANG
    //    ground frame
    //    non-stable mode
    //
    //  ends up being: 0b00100000 -> 0x20

    // clang-format off
    this->dji->attitude_control(
      0x20,                    // control mode byte (see above comment)
      rad2deg(euler(0)),       // roll (deg)
      rad2deg(euler(1)),       // pitch (deg)
      att_cmd.throttle * 100,  // throttle (0 - 100)
      rad2deg(euler(2))        // yaw (deg)
    );
    // clang-format on

  } else {
    ROS_ERROR("Invalid [fcu_type]: %s", this->fcu_type.c_str());
  }

}

void ControlNode::publishQuadrotorPose(void) {
  geometry_msgs::PoseStamped msg;
  buildMsg(this->ros_seq, ros::Time::now(), this->quadrotor.pose, msg);
  this->ros_pubs[QUADROTOR_POSE].publish(msg);
}

void ControlNode::publishQuadrotorVelocity(void) {
  geometry_msgs::TwistStamped msg;

  msg.twist.linear.x = this->quadrotor.velocity(0);
  msg.twist.linear.y = this->quadrotor.velocity(1);
  msg.twist.linear.z = this->quadrotor.velocity(2);

  this->ros_pubs[QUADROTOR_VELOCITY].publish(msg);
}

void ControlNode::publishPX4DummyMsg(void) {
  geometry_msgs::PoseStamped msg;

  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 2.0;

  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  this->ros_pubs[PX4_SETPOINT_POSITION_TOPIC].publish(msg);
}

int ControlNode::loopCallback(void) {
  double dt;

  // setup
  dt = (ros::Time::now() - this->ros_last_updated).toSec();

  // publish pose and velocity
  this->publishQuadrotorPose();
  this->publishQuadrotorVelocity();

  // step
  if (this->quadrotor.step(dt) != 0) {
    return -1;
  } else if (this->quadrotor.current_mode == DISARM_MODE) {
    this->setEstimatorOff();
  }

  // publish msgs
  if (this->armed || this->sim_mode) {
    this->publishAttitudeSetpoint();
    this->publishStats();
  } else if (this->fcu_type == "PX4") {
    this->publishPX4DummyMsg();
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
  // this->ros_pubs[PCTRL_STATS_TOPIC].publish(pc_stats_msg);
  // this->ros_pubs[PCTRL_GET].publish(pcs_settings_msg);
}

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::ControlNode, NODE_NAME, NODE_RATE);
