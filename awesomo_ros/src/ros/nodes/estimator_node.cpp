#include "awesomo_ros/nodes/estimator_node.hpp"

namespace awesomo {

int EstimatorNode::configure(std::string node_name, int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // set quad pose to zero so that when mavros is not active, this node still produces output
  this->quad_pose.position  << 0.0, 0.0, 0.0;
  this->quad_pose.orientation.w() = 0.0;
  this->quad_pose.orientation.x() = 0.0;
  this->quad_pose.orientation.y() = 0.0;
  this->quad_pose.orientation.z() = 0.0;

  // estimator
  ROS_GET_PARAM("/tracker_mode", this->mode);
  ROS_GET_PARAM("/quad_frame", this->quad_frame);
  this->initialized = false;
  this->state = ESTIMATOR_OFF;

  switch (this->mode) {
    case KF_MODE:
      log_info("Estimator running in KF_MODE!");
      ROS_GET_PARAM("/kf_tracker_config", config_file);
      if (this->kf_tracker.configure(config_file) != 0) {
        log_err("Failed to configure KalmanFilterTracker!");
        return -2;
      }
      break;

    case EKF_MODE:
      log_info("Estimator running in EKF_MODE!");
      ROS_GET_PARAM("/ekf_tracker_config", config_file);
      if (this->ekf_tracker.configure(config_file) != 0) {
        log_err("Failed to configure ExtendedKalmanFilterTracker!");
        return -2;
      }
      break;

    default:
      log_err("Invalid Tracker Mode!");
      return -2;
  }

  // publishers and subscribers
  // clang-format off
  this->registerPublisher<geometry_msgs::Vector3>(LT_INERTIAL_POSITION_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(LT_INERTIAL_VELOCITY_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(LT_BODY_POSITION_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(LT_BODY_VELOCITY_TOPIC);
  this->registerPublisher<std_msgs::Bool>(LT_DETECTED_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(GIMBAL_SETPOINT_ATTITUDE_TOPIC);
  this->registerPublisher<std_msgs::Float64>(QUAD_YAW_TOPIC);
  this->registerSubscriber(ESTIMATOR_ON_TOPIC, &EstimatorNode::onCallback, this);
  this->registerSubscriber(ESTIMATOR_OFF_TOPIC, &EstimatorNode::offCallback, this);
  this->registerSubscriber(QUAD_POSE_TOPIC, &EstimatorNode::quadPoseCallback, this);
  this->registerSubscriber(QUAD_VELOCITY_TOPIC, &EstimatorNode::quadVelocityCallback, this);
  this->registerSubscriber(TARGET_IF_POS_TOPIC, &EstimatorNode::targetInertialPosCallback, this);
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

      log_info("Intializing KF!");
      if (this->kf_tracker.initialize(mu) != 0) {
        log_err("Failed to intialize KalmanFilterTracker!");
        exit(-1);  // dangerous but necessary
      }
      break;

    case EKF_MODE:
      // clang-format off
      mu = VecX(9);
      mu << x0(0), x0(1), x0(2),
            0, 0, 0,
            0, 0, 0;
      // clang-format on

      log_info("Intializing EKF!");
      if (this->ekf_tracker.initialize(mu) != 0) {
        log_err("Failed to intialize ExtendedKalmanFilterTracker!");
        exit(-1);  // dangerous but necessary
      }
      break;
  }

  log_info("Estimator intialized!");
  this->initialized = true;
}

void EstimatorNode::resetLTKF(Vec3 x0) {
  this->initLTKF(x0);
}

void EstimatorNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
  convertMsg(msg, this->quad_pose);
}

void EstimatorNode::quadVelocityCallback(const geometry_msgs::TwistStamped &msg) {
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

void EstimatorNode::targetInertialPosCallback(const geometry_msgs::Vector3 &msg) {
  // pre-check
  if (this->state == ESTIMATOR_OFF) {
    return;
  }

  // update target
  this->target_detected = true;
  this->target_losted = false;
  convertMsg(msg, this->target_pos_wf);
  tic(&this->target_last_updated);

  // initialize estimator
  if (this->initialized == false && this->target_pos_x_init.size() == 10) {
    // calculate target position median in x, y, z
    // assuming quadrotor has not moved much
    this->target_pos_wf(0) = median(this->target_pos_x_init);
    this->target_pos_wf(1) = median(this->target_pos_y_init);
    this->target_pos_wf(2) = median(this->target_pos_z_init);

    this->initLTKF(this->target_pos_wf);

    this->target_pos_x_init.clear();
    this->target_pos_y_init.clear();
    this->target_pos_z_init.clear();

  // observe target position
  } else if (this->initialized == false && this->target_pos_x_init.size() < 10) {
    this->target_pos_x_init.push_back(this->target_pos_wf(0));
    this->target_pos_y_init.push_back(this->target_pos_wf(1));
    this->target_pos_z_init.push_back(this->target_pos_wf(2));

    log_info("Observed target position: %.2f, %.2f, %.2f", this->target_pos_wf(0),
                                                           this->target_pos_wf(1),
                                                           this->target_pos_wf(2));

  }

}

void EstimatorNode::targetInertialYawCallback(const std_msgs::Float64 &msg) {
  convertMsg(msg, this->target_yaw_wf);
}

void EstimatorNode::publishLTKFInertialPositionEstimate(void) {
  geometry_msgs::Vector3 msg;

  switch (this->mode) {
    case KF_MODE:
      msg.x = this->kf_tracker.mu(0);
      msg.y = this->kf_tracker.mu(1);
      msg.z = this->kf_tracker.mu(2);
      break;

    case EKF_MODE:
      msg.x = this->ekf_tracker.mu(0);
      msg.y = this->ekf_tracker.mu(1);
      msg.z = this->ekf_tracker.mu(2);
      break;
  }

  this->ros_pubs[LT_INERTIAL_POSITION_TOPIC].publish(msg);
}

void EstimatorNode::publishLTKFInertialVelocityEstimate(void) {
  geometry_msgs::Vector3 msg;

  switch (this->mode) {
    case KF_MODE:
      msg.x = this->kf_tracker.mu(3);
      msg.y = this->kf_tracker.mu(4);
      msg.z = this->kf_tracker.mu(5);
      break;

    case EKF_MODE:
      msg.x = this->ekf_tracker.mu(4) * cos(this->ekf_tracker.mu(3));
      msg.y = this->ekf_tracker.mu(4) * sin(this->ekf_tracker.mu(3));
      msg.z = this->ekf_tracker.mu(5);
      break;
  }

  this->ros_pubs[LT_INERTIAL_VELOCITY_TOPIC].publish(msg);
}

void EstimatorNode::publishLTKFBodyPositionEstimate(void) {
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
  // transform target position from inertial frame to body planar frame
  target2bodyplanar(est_pos,
                    this->quad_pose.position,
                    this->quad_pose.orientation,
                    this->target_pos_bpf);

  // build and publish msg
  buildMsg(this->target_pos_bpf, msg);
  this->ros_pubs[LT_BODY_POSITION_TOPIC].publish(msg);
}

void EstimatorNode::publishLTKFBodyVelocityEstimate(void) {
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
    std::cout << "Estimator ON" << std::endl;
  }

  // transform target velocity from inertial frame to body planar frame
  target2bodyplanar(est_vel,
                    this->quad_velocity,
                    this->quad_pose.orientation,
                    this->target_vel_bpf);

  // build and publish msg
  buildMsg(this->target_vel_bpf, msg);
  this->ros_pubs[LT_BODY_VELOCITY_TOPIC].publish(msg);
}

void EstimatorNode::publishLTDetected(void) {
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

void EstimatorNode::publishQuadYawMsg(void) {
  std_msgs::Float64 msg;

  // pre-check
  if (this->initialized == false) {
    return;
  }

  // build and publish msg
  buildMsg(this->target_yaw_wf, msg);
  this->ros_pubs[QUAD_YAW_TOPIC].publish(msg);
}

void EstimatorNode::trackTarget(void) {
  double dist;
  Vec3 setpoints;

  // calculate roll pitch yaw setpoints
  dist = this->target_pos_bpf.norm();
  setpoints(0) = asin(this->target_pos_bpf(1) / dist);   // roll
  setpoints(1) = -asin(this->target_pos_bpf(0) / dist);  // pitch
  setpoints(2) = 0.0;                                    // yaw

  this->publishGimbalSetpointAttitudeMsg(setpoints);
  this->publishQuadYawMsg();
}

void EstimatorNode::reset(void) {
  Vec3 setpoints;

  // reset estimator
  this->initialized = false;
  this->target_losted = true;
  this->target_detected = false;
  this->target_pos_wf << 0.0, 0.0, 0.0;

  // reset gimbal
  setpoints << 0, 0, 0;
  this->publishGimbalSetpointAttitudeMsg(setpoints);
}

int EstimatorNode::estimateKF(double dt) {
  MatX A(9, 9), C(3, 9);
  Vec3 y, prev_pos, curr_pos;

  // transition matrix - constant acceleration
  MATRIX_A_CONSTANT_ACCELERATION_XYZ(A);

  // setup
  prev_pos = this->target_last_pos_wf;

  // check measurement
  if (this->target_detected) {
    // clang-format off
    C << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    y << this->target_pos_wf(0),
         this->target_pos_wf(1),
         this->target_pos_wf(2);
    this->target_last_pos_wf = this->target_pos_wf;
    // clang-format on

  } else {
    // clang-format off
    C << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // clang-format on
    y << 0.0, 0.0, 0.0;  // to prevent uninitialized values from entering kf

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
  y(0) = this->target_pos_wf(0);
  y(1) = this->target_pos_wf(1);
  y(2) = this->target_pos_wf(2);
  y(3) = deg2rad(wrapTo180(rad2deg(this->target_yaw_wf)));

  // prediction update
  two_wheel_process_model(this->ekf_tracker, G, g, dt);
  this->ekf_tracker.predictionUpdate(g, G);

  // measurement update
  if (this->target_detected) {
    H(0, 0) = 1.0;  /* x */
    H(1, 1) = 1.0;  /* y */
    H(2, 2) = 1.0;  /* z */
    H(3, 3) = 1.0;  /* theta */
    h = H * this->ekf_tracker.mu_p;
    this->ekf_tracker.measurementUpdate(h, H, y);

  } else {
    this->ekf_tracker.mu = this->ekf_tracker.mu_p;

  }

  return 0;
}

int EstimatorNode::estimate(void) {
  double dt;
  int retval;

  // pre-check
  if (this->initialized == false) {
    return 0;
  }

  // check if target is losted
  if (mtoc(&this->target_last_updated) > this->target_lost_threshold) {
    log_info("Target lost, resetting estimator!");
    this->reset();
    return -1;
  }

  // setup
  dt = (ros::Time::now() - this->ros_last_updated).toSec();

  // estimate
  switch (this->mode) {
    case KF_MODE: retval = this->estimateKF(dt); break;
    case EKF_MODE: retval = this->estimateEKF(dt); break;
  }

  // sanity check target estimates
  if (retval != 0) {
    log_info("Bad estimates, resetting estimator!");
    this->reset();
    return -2;
  }

  return 0;
}

int EstimatorNode::loopCallback(void) {
  // pre-check
  if (this->initialized == false) {
    return 0;
  }

  // estimate and publish
  if (this->initialized && this->estimate() == 0) {
    this->publishLTKFInertialPositionEstimate();
    this->publishLTKFInertialVelocityEstimate();
    this->publishLTKFBodyPositionEstimate();
    this->publishLTKFBodyVelocityEstimate();
    this->trackTarget();
  }
  this->publishLTDetected();

  // reset
  this->target_detected = false;
  this->target_pos_wf << 0.0, 0.0, 0.0;

  return 0;
}

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::EstimatorNode, NODE_NAME, NODE_RATE);
