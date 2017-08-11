#include "atl/ros/nodes/estimator_node.hpp"

namespace atl {

int EstimatorNode::configure(int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  this->quad_pose = Pose();
  this->quad_velocity << 0, 0, 0;

  // estimator
  ROS_GET_PARAM("/estimator/tracker_mode", this->mode);
  ROS_GET_PARAM("/quad_frame", this->quad_frame);
  this->initialized = false;
  this->state = ESTIMATOR_OFF;

  switch (this->mode) {
  case KF_MODE:
    LOG_INFO("Estimator running in KF_MODE!");
    ROS_GET_PARAM("/estimator/kf/config", config_file);
    if (this->kf_tracker.configure(config_file) != 0) {
      LOG_ERROR("Failed to configure KalmanFilterTracker!");
      return -2;
    }
    break;

  case EKF_MODE:
    LOG_INFO("Estimator running in EKF_MODE!");
    ROS_GET_PARAM("/estimator/ekf/config", config_file);
    if (this->ekf_tracker.configure(config_file) != 0) {
      LOG_ERROR("Failed to configure ExtendedKalmanFilterTracker!");
      return -2;
    }
    break;

  default:
    LOG_ERROR("Invalid Tracker Mode!");
    return -2;
  }

  // publishers and subscribers
  // clang-format off
  this->registerPublisher<geometry_msgs::Vector3>(LT_BODY_POSITION_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(LT_BODY_VELOCITY_TOPIC);
  this->registerPublisher<std_msgs::Bool>(LT_DETECTED_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(GIMBAL_SETPOINT_ATTITUDE_TOPIC);
  this->registerPublisher<std_msgs::Float64>(QUAD_YAW_TOPIC);
  this->registerSubscriber(ESTIMATOR_ON_TOPIC, &EstimatorNode::onCallback, this);
  this->registerSubscriber(ESTIMATOR_OFF_TOPIC, &EstimatorNode::offCallback, this);
  this->registerSubscriber(QUAD_POSE_TOPIC, &EstimatorNode::quadPoseCallback, this);
  this->registerSubscriber(QUAD_VELOCITY_TOPIC, &EstimatorNode::quadVelocityCallback, this);
  this->registerSubscriber(TARGET_BF_POS_TOPIC, &EstimatorNode::targetBodyPosCallback, this);
  this->registerSubscriber(TARGET_IF_YAW_TOPIC, &EstimatorNode::targetInertialYawCallback, this);
  this->registerLoopCallback(std::bind(&EstimatorNode::loopCallback, this));
  // clang-format on

  this->configured = true;
  return 0;
}

void EstimatorNode::initLTKF(Vec3 x0) {
  VecX mu;

  // pre-check
  if (this->state == ESTIMATOR_OFF) {
    return;
  }

  // initialize estimator
  switch (this->mode) {
  case KF_MODE:
    // clang-format off
      mu = VecX(9);
      mu << x0(0), x0(1), x0(2),
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0;
    // clang-format on

    LOG_INFO("Intializing KF!");
    if (this->kf_tracker.initialize(mu) != 0) {
      LOG_ERROR("Failed to intialize KalmanFilterTracker!");
      exit(-1); // dangerous but necessary
    }
    break;

  case EKF_MODE:
    // clang-format off
      mu = VecX(9);
      mu << x0(0), x0(1), x0(2),
            0, 0, 0,
            0, 0, 0;
    // clang-format on

    LOG_INFO("Intializing EKF!");
    if (this->ekf_tracker.initialize(mu) != 0) {
      LOG_ERROR("Failed to intialize ExtendedKalmanFilterTracker!");
      exit(-1); // dangerous but necessary
    }
    break;
  }

  LOG_INFO("Estimator intialized!");
  this->initialized = true;
}

void EstimatorNode::resetLTKF(Vec3 x0) { this->initLTKF(x0); }

void EstimatorNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
  convertMsg(msg, this->quad_pose);
}

void EstimatorNode::quadVelocityCallback(
    const geometry_msgs::TwistStamped &msg) {
  convertMsg(msg.twist.linear, this->quad_velocity);
}

void EstimatorNode::onCallback(const std_msgs::Bool &msg) {
  bool data;

  // setup
  convertMsg(msg, data);

  // switch on estimator
  if (data) {
    this->state = ESTIMATOR_ON;
  }
}

void EstimatorNode::offCallback(const std_msgs::Bool &msg) {
  bool data;
  Vec3 setpoints;

  // setup
  convertMsg(msg, data);
  setpoints << 0.0, 0.0, 0.0;

  // switch off estimator
  if (data) {
    this->state = ESTIMATOR_OFF;
    this->initialized = false;
    this->publishGimbalSetpointAttitudeMsg(setpoints);
  }
}

void EstimatorNode::targetBodyPosCallback(const geometry_msgs::Vector3 &msg) {
  // pre-check
  if (this->state == ESTIMATOR_OFF) {
    return;
  }

  // update target
  this->target_detected = true;
  this->target_losted = false;
  convertMsg(msg, this->target_measured);
  tic(&this->target_last_updated);

  // initialize estimator
  if (this->initialized == false) {
    this->initLTKF(this->target_measured);
  }
}

void EstimatorNode::targetInertialPosCallback(
    const geometry_msgs::Vector3 &msg) {
  // pre-check
  if (this->state == ESTIMATOR_OFF) {
    return;
  }

  // update target
  this->target_detected = true;
  this->target_losted = false;
  convertMsg(msg, this->target_measured);
  tic(&this->target_last_updated);

  // initialize estimator
  if (this->initialized == false) {
    this->initLTKF(this->target_measured);
  }
}

void EstimatorNode::targetInertialYawCallback(const std_msgs::Float64 &msg) {
  convertMsg(msg, this->target_yaw_wf);
}

void EstimatorNode::publishLTKFBodyPositionEstimate() {
  geometry_msgs::Vector3 msg;
  Vec3 est_pos;

  // setup
  switch (this->mode) {
  case KF_MODE:
    est_pos(0) = this->kf_tracker.mu(0);
    est_pos(1) = this->kf_tracker.mu(1);
    est_pos(2) = this->kf_tracker.mu(2);
    break;

  case EKF_MODE:
    est_pos(0) = this->ekf_tracker.mu(0);
    est_pos(1) = this->ekf_tracker.mu(1);
    est_pos(2) = this->ekf_tracker.mu(2);
    break;
  }

  // estimate in body planar frame
  this->target_pos_bpf(0) = est_pos(0);
  this->target_pos_bpf(1) = est_pos(1);
  this->target_pos_bpf(2) = est_pos(2);

  // build and publish msg
  buildMsg(this->target_pos_bpf, msg);
  this->ros_pubs[LT_BODY_POSITION_TOPIC].publish(msg);
}

void EstimatorNode::publishLTKFBodyVelocityEstimate() {
  geometry_msgs::Vector3 msg;
  Vec3 est_vel;

  // setup
  switch (this->mode) {
  case KF_MODE:
    est_vel(0) = this->kf_tracker.mu(3);
    est_vel(1) = this->kf_tracker.mu(4);
    est_vel(2) = this->kf_tracker.mu(5);
    break;

  case EKF_MODE:
    est_vel(0) = this->ekf_tracker.mu(4) * cos(this->ekf_tracker.mu(3));
    est_vel(1) = this->ekf_tracker.mu(4) * sin(this->ekf_tracker.mu(3));
    est_vel(2) = this->ekf_tracker.mu(5);
    break;
  }
  std::cout << "Estimator ON" << std::endl;

  // estimate in body planar frame
  this->target_vel_bpf(0) = est_vel(0);
  this->target_vel_bpf(1) = est_vel(1);
  this->target_vel_bpf(2) = est_vel(2);

  // build and publish msg
  buildMsg(this->target_vel_bpf, msg);
  this->ros_pubs[LT_BODY_VELOCITY_TOPIC].publish(msg);
}

void EstimatorNode::publishLTDetected() {
  std_msgs::Bool msg;
  if (this->target_losted) {
    msg.data = false;
  } else {
    msg.data = true;
  }
  this->ros_pubs[LT_DETECTED_TOPIC].publish(msg);
}

void EstimatorNode::publishGimbalSetpointAttitudeMsg(Vec3 setpoints) {
  geometry_msgs::Vector3 msg;
  buildMsg(setpoints, msg);
  this->ros_pubs[GIMBAL_SETPOINT_ATTITUDE_TOPIC].publish(msg);
}

void EstimatorNode::publishQuadYawMsg() {
  std_msgs::Float64 msg;

  // pre-check
  if (this->initialized == false) {
    return;
  }

  // build and publish msg
  buildMsg(this->target_yaw_wf, msg);
  this->ros_pubs[QUAD_YAW_TOPIC].publish(msg);
}

void EstimatorNode::trackTarget() {
  double dist;
  Vec3 setpoints;

  // calculate roll pitch yaw setpoints
  dist = this->target_pos_bpf.norm();
  setpoints(0) = asin(this->target_pos_bpf(1) / dist);  // roll
  setpoints(1) = -asin(this->target_pos_bpf(0) / dist); // pitch
  setpoints(2) = 0.0;                                   // yaw

  this->publishGimbalSetpointAttitudeMsg(setpoints);
  this->publishQuadYawMsg();
}

void EstimatorNode::reset() {
  Vec3 setpoints;

  // reset estimator
  this->initialized = false;
  this->target_losted = true;
  this->target_detected = false;
  this->target_measured << 0.0, 0.0, 0.0;

  // reset gimbal
  setpoints << 0, 0, 0;
  this->publishGimbalSetpointAttitudeMsg(setpoints);
}

int EstimatorNode::estimateKF(double dt) {
  MatX A(9, 9), C(3, 9);
  Vec3 y, prev_pos, curr_pos;

  // setup
  prev_pos = this->target_last_measured;
  MATRIX_A_CONSTANT_ACCELERATION_XYZ(A);

  // check measurement
  if (this->target_detected) {
    // clang-format off
    C << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    y << this->target_measured(0),
         this->target_measured(1),
         this->target_measured(2);
    this->target_last_measured = this->target_measured;
    // clang-format on

  } else {
    // clang-format off
    C << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // clang-format on
    y << 0.0, 0.0, 0.0; // to prevent uninitialized values from entering kf
  }

  // estimate
  this->kf_tracker.C = C;
  this->kf_tracker.estimate(A, y);

  // sanity check estimates
  curr_pos = this->kf_tracker.mu.block(0, 0, 3, 1);
  if (this->kf_tracker.sanityCheck(prev_pos, curr_pos) == -2) {
    return -1;
  }

  return 0;
}

int EstimatorNode::estimateEKF(double dt) {
  VecX y(4), g(9), h(4);
  MatX G(9, 9), H(4, 9);

  // setup
  H = MatX::Zero(4, 9);
  y(0) = this->target_measured(0);
  y(1) = this->target_measured(1);
  y(2) = this->target_measured(2);
  y(3) = deg2rad(wrapTo180(rad2deg(this->target_yaw_wf)));

  // prediction update
  two_wheel_process_model(this->ekf_tracker, G, g, dt);
  this->ekf_tracker.predictionUpdate(g, G);

  // measurement update
  if (this->target_detected) {
    H(0, 0) = 1.0; // x
    H(1, 1) = 1.0; // y
    H(2, 2) = 1.0; // z
    H(3, 3) = 1.0; // theta
    h = H * this->ekf_tracker.mu_p;
    this->ekf_tracker.measurementUpdate(h, H, y);

  } else {
    this->ekf_tracker.mu = this->ekf_tracker.mu_p;
  }

  return 0;
}

int EstimatorNode::estimate() {
  int retval;

  // pre-check
  if (this->initialized == false) {
    return 0;
  }

  // check if target is losted
  if (mtoc(&this->target_last_updated) > this->target_lost_threshold) {
    LOG_INFO("Target lost, resetting estimator!");
    this->reset();
    return -1;
  }

  // setup
  double dt = (ros::Time::now() - this->ros_last_updated).toSec();

  // estimate
  switch (this->mode) {
  case KF_MODE:
    retval = this->estimateKF(dt);
    break;
  case EKF_MODE:
    retval = this->estimateEKF(dt);
    break;
  }

  // sanity check target estimates
  if (retval != 0) {
    LOG_INFO("Bad estimates, resetting estimator!");
    this->reset();
    return -2;
  }

  return 0;
}

int EstimatorNode::loopCallback() {
  // pre-check
  if (this->initialized == false) {
    return 0;
  }

  // estimate and publish
  if (this->initialized && this->estimate() == 0) {
    this->publishLTKFBodyPositionEstimate();
    this->publishLTKFBodyVelocityEstimate();
    this->trackTarget();
  }
  this->publishLTDetected();

  // reset
  this->target_detected = false;
  this->target_measured << 0.0, 0.0, 0.0;

  return 0;
}

} // namespace atl

RUN_ROS_NODE(atl::EstimatorNode, NODE_RATE);
