#include "atl/ros/nodes/control_node.hpp"

namespace atl {

int ControlNode::configure(const std::string node_name, int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // quadrotor
  ROS_GET_PARAM("/quad_frame", this->quad_frame);
  ROS_GET_PARAM("/fcu_type", this->fcu_type);
  ROS_GET_PARAM("/control/config_dir", config_path);
  if (this->quadrotor.configure(config_path) != 0) {
    ROS_ERROR(FCONFQUAD);
    return -2;
  }

  // configure PX4 or DJI topics
  if (this->fcu_type == "PX4") {
    this->configurePX4Topics();

  } else if (this->fcu_type == "DJI") {
    this->configureDJITopics();

  } else {
    ROS_ERROR("Invalid [fcu_type]: %s", this->fcu_type.c_str());
    return -1;
  }

  // publishers
  this->registerPublisher<atl_msgs::PCtrlSettings>(PCTRL_GET_TOPIC);
  this->registerPublisher<geometry_msgs::PoseStamped>(QUADROTOR_POSE);
  this->registerPublisher<geometry_msgs::TwistStamped>(QUADROTOR_VELOCITY);
  this->registerPublisher<std_msgs::Bool>(ESTIMATOR_ON_TOPIC);
  this->registerPublisher<std_msgs::Bool>(ESTIMATOR_OFF_TOPIC);

  // subscribers
  // clang-format off
  this->registerSubscriber(ARM_TOPIC, &ControlNode::armCallback, this);
  this->registerSubscriber(MODE_TOPIC, &ControlNode::modeCallback, this);
  this->registerSubscriber(YAW_TOPIC, &ControlNode::yawCallback, this);
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
  // this->waitForEstimator();

  this->configured = true;
  return 0;
}

int ControlNode::configurePX4Topics() {
  // clang-format off
  // services
  this->registerClient<mavros_msgs::SetMode>(PX4_MODE_TOPIC);
  this->registerClient<mavros_msgs::CommandBool>(PX4_ARM_TOPIC);

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

int ControlNode::configureDJITopics() {
  // clang-format off
  // services
  this->registerClient<dji_sdk::DroneArmControl>(DJI_ARM_TOPIC);
  this->registerClient<dji_sdk::SDKControlAuthority>(DJI_SDK_AUTH_TOPIC);

  // publishers
  this->registerPublisher<sensor_msgs::Joy>(DJI_SETPOINT_TOPIC);

  // subscribers
  this->registerSubscriber(DJI_GPS_POSITION_TOPIC, &ControlNode::djiGPSPositionCallback, this);
  this->registerSubscriber(DJI_ATTITUDE_TOPIC, &ControlNode::djiAttitudeCallback, this);
  this->registerSubscriber(DJI_VELOCITY_TOPIC, &ControlNode::djiVelocityCallback, this);
  this->registerSubscriber(DJI_RADIO_TOPIC, &ControlNode::djiRadioCallback, this);
  // clang-format on

  return 0;
}

int ControlNode::px4Connect() {
  int attempts;

  // pre-check
  if (this->sim_mode) {
    return 0;
  }

  // wait fo FCU
  attempts = 0;
  LOG_INFO("Waiting for PX4 FCU ...");
  while (this->px4_state.connected != true) {
    if (attempts == 10) {
      LOG_INFO("Failed to connect to PX4 FCU for 10 seconds ...");
      return -1;
    }

    sleep(1);
    ros::spinOnce();
    attempts++;
  }

  LOG_INFO("Connected to PX4 FCU!");
  return 0;
}

int ControlNode::px4Disarm() {
  mavros_msgs::CommandBool msg;

  // setup
  LOG_INFO("Disarming atl ...");
  msg.request.value = false;

  // disarm
  if (this->ros_clients[PX4_ARM_TOPIC].call(msg) && msg.response.success) {
    LOG_INFO("PX4 FCU disarmed!");
    return 0;
  } else {
    LOG_ERROR("Failed to disarm PX4 FCU!");
    return -1;
  }
}

int ControlNode::px4OffboardModeOn() {
  mavros_msgs::SetMode msg;
  msg.request.custom_mode = "OFFBOARD";

  if (this->ros_clients[PX4_MODE_TOPIC].call(msg) && msg.response.success) {
    LOG_INFO("PX4 FCU OFFBOARD MODE ON!");
    return 0;
  } else {
    LOG_ERROR("Failed to enable PX4 FCU offboard mode!");
    return -1;
  }
}

int ControlNode::djiDisarm() {
  dji_sdk::DroneArmControl msg;

  // send disarm msg
  msg.request.arm = false;
  bool msg_sent = this->ros_clients[DJI_ARM_TOPIC].call(msg);
  if (msg_sent == true && msg.response.result == true) {
    LOG_ERROR("Failed to disarm DJI FCU!");
    return -1;
  }

  LOG_INFO("DJI FCU DISARMED!");
  return 0;
}

int ControlNode::djiOffboardModeOn() {
  dji_sdk::SDKControlAuthority msg;

  msg.request.control_enable = 1;
  bool msg_sent = this->ros_clients[DJI_SDK_AUTH_TOPIC].call(msg);
  if (msg_sent == true && msg.response.result == true) {
    LOG_ERROR("Failed to gain DJI SDK control!");
    return -1;
  }

  LOG_INFO("DJI SDK control obtained!");
  return 0;
}

int ControlNode::djiOffboardModeOff() {
  dji_sdk::SDKControlAuthority msg;

  msg.request.control_enable = 0;
  bool msg_sent = this->ros_clients[DJI_SDK_AUTH_TOPIC].call(msg);
  if (msg_sent == true && msg.response.result == true) {
    LOG_ERROR("Failed to release DJI SDK control!");
    return -1;
  }

  LOG_INFO("DJI SDK control released!");
  return 0;
}

int ControlNode::waitForEstimator() {
  int attempts;

  // wait for estimator
  LOG_INFO("Waiting for Estimator ...");
  attempts = 0;

  while (this->ros_pubs[ESTIMATOR_ON_TOPIC].getNumSubscribers() == 0) {
    if (attempts == 10) {
      LOG_INFO("Failed to connect to EstimatorNode for 10 seconds ...");
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

void ControlNode::setEstimatorOn() {
  std_msgs::Bool msg;
  msg.data = true;
  this->ros_pubs[ESTIMATOR_ON_TOPIC].publish(msg);
}

void ControlNode::setEstimatorOff() {
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
    LOG_ERROR("Invalid ROS [/quad_frame] param value: %s",
              this->quad_frame.c_str());
  }

  this->quadrotor.setPose(pose);
}

void ControlNode::px4VelocityCallback(
  const geometry_msgs::TwistStamped &msg) {
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
    LOG_ERROR("Invalid ROS [/quad_frame] param value: %s",
              this->quad_frame.c_str());
  }
}

void ControlNode::px4RadioCallback(const mavros_msgs::RCIn &msg) {
  int rc_in[16];

  // setup
  for (int i = 0; i < 16; i++) {
    rc_in[i] = msg.channels[i];
  }

  // parse
  if (this->armed) {
    if (rc_in[6] < 1500) {
      this->armed = false;
      this->quadrotor.setMode(HOVER_MODE);
    }
  } else {
    if (rc_in[6] > 1500) {
      this->armed = true;
      this->quadrotor.setMode(DISCOVER_MODE);
    }
  }
}

void ControlNode::djiGPSPositionCallback(const sensor_msgs::NavSatFix &msg) {
  // set home point
  if (this->home_set == false) {
    this->home_lat = msg.latitude;
    this->home_lon = msg.longitude;
    this->home_alt = msg.altitude - 4.0;
    this->home_set = true;
  }

  // calculate local position
  double diff_N = 0.0;
  double diff_E = 0.0;
  double diff_alt = 0.0;
  latlon_diff(this->home_lat,
              this->home_lon,
              msg.latitude,
              msg.longitude,
              &diff_N,
              &diff_E);
  diff_alt = msg.altitude - this->home_alt;

  // set quadrotor position (ENU)
  this->quadrotor.pose.position << diff_E, diff_N, diff_alt;
}

void ControlNode::djiAttitudeCallback(
  const geometry_msgs::QuaternionStamped &msg) {
  Quaternion orientation;
  convertMsg(msg, orientation);
  this->quadrotor.pose.orientation = orientation;
}

void ControlNode::djiVelocityCallback(
  const geometry_msgs::Vector3Stamped &msg) {
  Vec3 vel;
  convertMsg(msg, vel);
  this->quadrotor.setVelocity(vel);
}

void ControlNode::djiRadioCallback(const sensor_msgs::Joy &msg) {
  int mode_switch = msg.axes[4];
  if (this->armed) {
    if (mode_switch < 0) {
      this->armed = false;
      this->djiOffboardModeOff();
      this->setEstimatorOff();
    }
  } else {
    if (mode_switch > 0) {
      this->armed = true;
      this->quadrotor.setMode(DISCOVER_MODE);
      this->djiOffboardModeOn();
      this->setEstimatorOn();
    }
  }
}

void ControlNode::armCallback(const std_msgs::Bool &msg) {
  if (msg.data == true) {
    this->armed = true;
    this->setEstimatorOn();

    if (this->fcu_type == "DJI" && this->sim_mode == false) {
      this->djiOffboardModeOn();
      this->quadrotor.setMode(HOVER_MODE);
    }

  } else {
    this->armed = false;
    this->setEstimatorOff();

    if (this->fcu_type == "DJI" && this->sim_mode == false) {
      this->djiOffboardModeOff();
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

void ControlNode::yawCallback(const std_msgs::Float64 &msg) {
  double yaw;
  convertMsg(msg, yaw);
  this->quadrotor.setYaw(yaw);
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
  const atl_msgs::PCtrlSettings &msg) {
  convertMsg(msg, this->quadrotor.position_controller);
}

void ControlNode::trackingControllerSetCallback(
  const atl_msgs::TCtrlSettings &msg) {
  convertMsg(msg, this->quadrotor.tracking_controller);
}

void ControlNode::landingControllerSetCallback(
  const atl_msgs::LCtrlSettings &msg) {
  // convertMsg(msg, this->quadrotor.landing_controller);
}

void ControlNode::publishAttitudeSetpoint() {
  // setup
  int seq = this->ros_seq;
  AttitudeCommand att_cmd = this->quadrotor.att_cmd;

  if (this->fcu_type == "PX4") {
    std_msgs::Float64 thr_msg;
    geometry_msgs::PoseStamped att_msg;
    buildMsg(seq, ros::Time::now(), att_cmd, att_msg, thr_msg);

    this->ros_pubs[PX4_SETPOINT_ATTITUDE_TOPIC].publish(att_msg);
    this->ros_pubs[PX4_SETPOINT_THROTTLE_TOPIC].publish(thr_msg);

  } else if (this->fcu_type == "DJI") {
    // transform orientation from NWU to NED
    Vec3 euler;
    Quaternion q_ned;
    nwu2ned(att_cmd.orientation, q_ned);
    quat2euler(q_ned, 321, euler);

    //  DJI Control Flag Byte
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
    sensor_msgs::Joy msg;
    msg.axes.push_back(rad2deg(euler(0)));       // roll (deg)
    msg.axes.push_back(rad2deg(euler(1)));       // pitch (deg)
    msg.axes.push_back(att_cmd.throttle * 100);  // throttle (0 - 100)
    msg.axes.push_back(rad2deg(euler(2)));       // yaw (deg)
    msg.axes.push_back(0x20);  // control flag (see above comment)
    this->ros_pubs[DJI_SETPOINT_TOPIC].publish(msg);

  } else {
    ROS_ERROR("Invalid [fcu_type]: %s", this->fcu_type.c_str());
    exit(-1);  // dangerous but necessary
  }
}

void ControlNode::publishQuadrotorPose() {
  geometry_msgs::PoseStamped msg;
  buildMsg(this->ros_seq, ros::Time::now(), this->quadrotor.pose, msg);
  this->ros_pubs[QUADROTOR_POSE].publish(msg);
}

void ControlNode::publishQuadrotorVelocity() {
  geometry_msgs::TwistStamped msg;

  msg.twist.linear.x = this->quadrotor.velocity(0);
  msg.twist.linear.y = this->quadrotor.velocity(1);
  msg.twist.linear.z = this->quadrotor.velocity(2);

  this->ros_pubs[QUADROTOR_VELOCITY].publish(msg);
}

int ControlNode::loopCallback() {
  double dt;

  // publish pose and velocity
  this->publishQuadrotorPose();
  this->publishQuadrotorVelocity();

  // pre-check
  if (this->armed == false) {
    this->quadrotor.reset();
    this->setEstimatorOff();
    return 0;
  }
  this->setEstimatorOn();

  // setup
  dt = (ros::Time::now() - this->ros_last_updated).toSec();

  // step
  if (this->quadrotor.step(dt) != 0) {
    return -1;
  } else if (this->quadrotor.current_mode == DISARM_MODE) {
    this->setEstimatorOff();
  }

  // publish msgs
  if (this->armed || this->sim_mode) {
    this->publishAttitudeSetpoint();
  }

  return 0;
}

}  // namespace atl

RUN_ROS_NODE(atl::ControlNode, NODE_NAME, NODE_RATE);
