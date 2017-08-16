#include "atl/ros/nodes/control_node.hpp"

namespace atl {

int ControlNode::configure(int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  // quadrotor
  ROS_GET_PARAM("/control/config_dir", config_path);
  this->dji = new DJIDrone(*this->ros_nh);
  if (this->quadrotor.configure(config_path) != 0) {
    ROS_ERROR(FCONFQUAD);
    return -2;
  }

  // publishers
  this->registerPublisher<atl_msgs::PCtrlSettings>(PCTRL_GET_TOPIC);
  this->registerPublisher<geometry_msgs::PoseStamped>(QUADROTOR_POSE);
  this->registerPublisher<geometry_msgs::TwistStamped>(QUADROTOR_VELOCITY);
  this->registerPublisher<std_msgs::Bool>(ESTIMATOR_ON_TOPIC);
  this->registerPublisher<std_msgs::Bool>(ESTIMATOR_OFF_TOPIC);

  // subscribers
  // clang-format off
  this->registerSubscriber(DJI_GPS_POSITION_TOPIC, &ControlNode::globalPositionCallback, this);
  this->registerSubscriber(DJI_LOCAL_POSITION_TOPIC, &ControlNode::localPositionCallback, this);
  this->registerSubscriber(DJI_ATTITUDE_TOPIC, &ControlNode::attitudeCallback, this);
  this->registerSubscriber(DJI_VELOCITY_TOPIC, &ControlNode::velocityCallback, this);
  this->registerSubscriber(DJI_RADIO_TOPIC, &ControlNode::radioCallback, this);

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

  // connect to estimator
  // this->waitForEstimator();

  this->configured = true;
  return 0;
}

int ControlNode::disarm() {
  if (this->dji->drone_disarm() != true) {
    return -1;
  }

  return 0;
}

int ControlNode::sdkControlMode(const bool mode) {
  if (mode) {
    if (this->dji->request_sdk_permission_control() != true) {
      LOG_ERROR("Failed to request DJI SDK control!");
      return -1;
    }
    LOG_INFO("Obtained DJI SDK control!");

  } else {
    if (this->dji->release_sdk_permission_control() != true) {
      LOG_ERROR("Failed to release DJI SDK control!");
      return -1;
    }
    LOG_INFO("Released DJI SDK control!");
  }

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

int ControlNode::waitForGPS() {
  int attempts;

  // wait for estimator
  LOG_INFO("Waiting for GPS ...");
  attempts = 0;

  while (this->home_set == false) {
    if (attempts == 10) {
      LOG_INFO("No GPS connection for 10 seconds ...");
      return -1;
    }

    ros::spinOnce();
    sleep(1);
    attempts++;
  }

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

void ControlNode::globalPositionCallback(const dji_sdk::GlobalPosition &msg) {
  this->latitude = msg.latitude;
  this->longitude = msg.longitude;

  if (this->home_set == false) {
    this->home_latitude = msg.latitude;
    this->home_longitude = msg.longitude;
    this->home_altitude = msg.altitude;
    this->home_set = true;

    this->quadrotor.setHomePoint(this->home_latitude, this->home_longitude);
  }
}

void ControlNode::localPositionCallback(const dji_sdk::LocalPosition &msg) {
  // convert msg from NED to ENU
  Vec3 pos_ned{msg.x, msg.y, msg.z};
  Vec3 pos_enu = ned2enu(pos_ned);
  this->quadrotor.pose.position = pos_enu;
}

void ControlNode::attitudeCallback(const dji_sdk::AttitudeQuaternion &msg) {
  Quaternion orientation_ned;

  orientation_ned.w() = msg.q0;
  orientation_ned.x() = msg.q1;
  orientation_ned.y() = msg.q2;
  orientation_ned.z() = msg.q3;

  // transform pose position and orientation
  // from NED to NWU
  Quaternion orientation_nwu = ned2nwu(orientation_ned);
  this->quadrotor.pose.orientation = orientation_nwu;
}

void ControlNode::velocityCallback(const dji_sdk::Velocity &msg) {
  // convert msg from NED to ENU
  Vec3 vel_ned{msg.vx, msg.vy, msg.vz};
  Vec3 vel_enu = ned2enu(vel_ned);
  this->quadrotor.setVelocity(vel_enu);
}

void ControlNode::radioCallback(const dji_sdk::RCChannels &msg) {
  if (msg.mode > 0 && this->armed == true) {
    this->armed = false;
    this->sdkControlMode(false);
    this->setEstimatorOff();

  } else if (msg.mode < 0 && this->armed == false) {
    this->armed = true;
    this->quadrotor.setMode(DISCOVER_MODE);
    this->sdkControlMode(true);
    this->setEstimatorOn();
  }
}

void ControlNode::armCallback(const std_msgs::Bool &msg) {
  if (msg.data == true) {
    this->armed = true;
    this->setEstimatorOn();

    if (this->sim_mode == false) {
      this->sdkControlMode(true);
      this->quadrotor.setMode(HOVER_MODE);
    }

  } else {
    this->armed = false;
    this->setEstimatorOff();

    if (this->sim_mode == false) {
      this->sdkControlMode(false);
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
  } else if (mode == "WAYPOINT_MODE") {
    this->quadrotor.setMode(WAYPOINT_MODE);
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
  AttitudeCommand att_cmd = this->quadrotor.att_cmd;
  const Vec3 rpy_ned = att_cmd.toEuler("NED");

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
  //    body frame
  //    non-stable mode
  //
  //  ends up being: 0b00100010 -> 0x22
  this->dji->attitude_control(0x22, // control mode byte (see above comment)
                              rad2deg(rpy_ned(0)),    // roll (deg)
                              rad2deg(rpy_ned(1)),    // pitch (deg)
                              att_cmd.throttle * 100, // throttle (0 - 100)
                              rad2deg(rpy_ned(2)));   // yaw (deg)
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

int ControlNode::takeoff() {
  // pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->armed == false) {
    return -2;
  }

  // takeoff
  double z = this->quadrotor.hover_position(2);
  this->quadrotor.hover_position = Vec3{0.0, 0.0, z};
  this->sdkControlMode(true);

  return 0;
}

int ControlNode::land() {
  // pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->armed == false) {
    return -2;
  }

  // land
  double x = this->quadrotor.pose.position(0);
  double y = this->quadrotor.pose.position(1);
  this->quadrotor.hover_position = Vec3{x, y, 0.0};

  return 0;
}

int ControlNode::loopCallback() {
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
  double dt = (ros::Time::now() - this->ros_last_updated).toSec();

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

} // namespace atl

RUN_ROS_NODE(atl::ControlNode, NODE_RATE);
