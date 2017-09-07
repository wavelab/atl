#include "atl/ros/nodes/control_node.hpp"

namespace atl {

int ControlNode::configure(const int hz) {
  std::string config_path;

  // Ros node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  // Quadrotor
  ROS_GET_PARAM(this->node_name + "/config_dir", config_path);
  if (this->quadrotor.configure(config_path) != 0) {
    ROS_ERROR(FCONFQUAD);
    return -2;
  }

  // Publishers
  this->addPublisher<atl_msgs::PCtrlSettings>(PCTRL_GET_TOPIC);
  this->addPublisher<geometry_msgs::PoseStamped>(QUADROTOR_POSE_TOPIC);
  this->addPublisher<geometry_msgs::TwistStamped>(QUADROTOR_VELOCITY_TOPIC);
  this->addPublisher<std_msgs::Bool>(ESTIMATOR_ON_TOPIC);
  this->addPublisher<std_msgs::Bool>(ESTIMATOR_OFF_TOPIC);
  this->addPublisher<sensor_msgs::Joy>(DJI_CONTROL_TOPIC);

  // Subscribers
  // clang-format off
  this->addSubscriber(DJI_GPS_POSITION_TOPIC, &ControlNode::globalPositionCallback, this);
  this->addSubscriber(DJI_ATTITUDE_TOPIC, &ControlNode::attitudeCallback, this);
  this->addSubscriber(DJI_VELOCITY_TOPIC, &ControlNode::velocityCallback, this);
  this->addSubscriber(DJI_RADIO_TOPIC, &ControlNode::radioCallback, this);

  this->addSubscriber(ARM_TOPIC, &ControlNode::armCallback, this);
  this->addSubscriber(MODE_TOPIC, &ControlNode::modeCallback, this);
  this->addSubscriber(YAW_TOPIC, &ControlNode::yawCallback, this);
  this->addSubscriber(TARGET_BODY_POSITION_TOPIC, &ControlNode::targetPositionCallback, this);
  this->addSubscriber(TARGET_BODY_VELOCITY_TOPIC, &ControlNode::targetVelocityCallback, this);
  this->addSubscriber(TARGET_DETECTED_TOPIC, &ControlNode::targetDetectedCallback, this);
  this->addSubscriber(HOVER_SET_TOPIC, &ControlNode::hoverSetCallback, this);
  this->addSubscriber(HOVER_HEIGHT_SET_TOPIC, &ControlNode::hoverHeightSetCallback, this);
  this->addSubscriber(PCTRL_SET_TOPIC, &ControlNode::positionControllerSetCallback, this);
  this->addSubscriber(TCTRL_SET_TOPIC, &ControlNode::trackingControllerSetCallback, this);
  this->addSubscriber(LCTRL_SET_TOPIC, &ControlNode::landingControllerSetCallback, this);
  // clang-format on

  // Services
  this->addClient<dji_sdk::SDKControlAuthority>(DJI_SDK_SERVICE);
  this->addClient<dji_sdk::DroneArmControl>(DJI_ARM_SERVICE);

  // Loop callback
  this->addLoopCallback(std::bind(&ControlNode::loopCallback, this));

  // Connect to estimator
  // this->waitForEstimator();

  this->configured = true;
  return 0;
}

int ControlNode::disarm() {
  // Request disarm
  dji_sdk::DroneArmControl msg;
  msg.request.arm = false;
  this->ros_clients[DJI_ARM_SERVICE].call(msg);

  // Check response
  if (msg.response.result == false) {
    LOG_ERROR("Failed to disarm quadrotor!");
    return -1;
  }
  LOG_INFO("Quadrotor disarmed!");

  return 0;
}

int ControlNode::sdkControlMode(const bool mode) {
  dji_sdk::SDKControlAuthority msg;
  msg.request.control_enable = mode;

  if (mode) {
    this->ros_clients[DJI_SDK_SERVICE].call(msg);
    if (msg.response.result == false) {
      LOG_ERROR("Failed to request DJI SDK control!");
      return -1;
    }
    LOG_INFO("Obtained DJI SDK control!");

  } else {
    this->ros_clients[DJI_SDK_SERVICE].call(msg);
    if (msg.response.result == false) {
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

void ControlNode::globalPositionCallback(const sensor_msgs::NavSatFix &msg) {
  this->gps_status = msg.status.status;
  this->gps_service = msg.status.service;
  this->latitude = msg.latitude;
  this->longitude = msg.longitude;

  // Set home position
  if (this->home_set == false) {
    this->home_latitude = msg.latitude;
    this->home_longitude = msg.longitude;
    this->home_altitude = msg.altitude;
    this->home_set = true;

    this->quadrotor.setHomePoint(this->home_latitude, this->home_longitude);
  }

  // Update local position relative to home point
  double dist_N = 0.0;
  double dist_E = 0.0;
  const double height = msg.altitude - this->home_altitude;
  latlon_diff(this->home_latitude,
              this->home_longitude,
              msg.latitude,
              msg.longitude,
              &dist_N,
              &dist_E);
  this->quadrotor.pose.position << dist_N, -1.0 * dist_E, height;
}

void ControlNode::attitudeCallback(
    const geometry_msgs::QuaternionStamped &msg) {
  // Convert ENU attitude to NWU attitude by adding 90 degrees to yaw
  Quaternion quat{msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z};
  Vec3 rpy = quatToEuler321(quat);
  rpy(2) -= M_PI / 2.0;
  quat = euler321ToQuat(rpy);

  this->quadrotor.pose.orientation = quat;
}

void ControlNode::velocityCallback(const geometry_msgs::Vector3Stamped &msg) {
  // Transform velocity from ENU to NWU
  Vec3 vel_enu{msg.vector.x, msg.vector.y, msg.vector.z};
  Vec3 vel_nwu = T_nwu_enu * vel_enu;
  this->quadrotor.setVelocity(vel_nwu);
}

void ControlNode::radioCallback(const sensor_msgs::Joy &msg) {
  const int mode_switch = msg.axes[4];

  // Arm or disarm SDK mode
  if (mode_switch > 0 && this->armed == true) {
    this->armed = false;
    this->sdkControlMode(false);
    this->setEstimatorOff();

  } else if (mode_switch < 0 && this->armed == false) {
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
  const int control_byte = 0x22;

  // Control signal in roll, pitch and yaw (radians) in ENU frame
  const Vec3 rpy = this->quadrotor.att_cmd.toEuler("ENU");

  // Throttle (0 - 100)
  const double throttle = this->quadrotor.att_cmd.throttle * 100.0;

  // Setup and publish control message
  sensor_msgs::Joy msg;
  msg.axes.push_back(rpy(0));       // roll (radians)
  msg.axes.push_back(rpy(1));       // pitch (radians)
  msg.axes.push_back(throttle);     // throttle (0 - 100)
  msg.axes.push_back(rpy(2));       // yaw (radians)
  msg.axes.push_back(control_byte); // control mode
  this->ros_pubs[DJI_CONTROL_TOPIC].publish(msg);
}

void ControlNode::publishQuadrotorPose() {
  geometry_msgs::PoseStamped msg;
  buildMsg(this->ros_seq, ros::Time::now(), this->quadrotor.pose, msg);
  this->ros_pubs[QUADROTOR_POSE_TOPIC].publish(msg);
}

void ControlNode::publishQuadrotorVelocity() {
  geometry_msgs::TwistStamped msg;

  msg.twist.linear.x = this->quadrotor.velocity(0);
  msg.twist.linear.y = this->quadrotor.velocity(1);
  msg.twist.linear.z = this->quadrotor.velocity(2);

  this->ros_pubs[QUADROTOR_VELOCITY_TOPIC].publish(msg);
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
  const double dt = (ros::Time::now() - this->ros_last_updated).toSec();

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
