#include "awesomo_ros/nodes/estimator_node.hpp"

namespace awesomo {

int EstimatorNode::configure(std::string node_name, int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // publishers and subscribers
  // clang-format off
  this->registerPublisher<geometry_msgs::Vector3>(LT_INERTIAL_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(LT_BODY_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(LT_VELOCITY_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(GIMBAL_SETPOINT_ATTITUDE_TOPIC);
  this->registerSubscriber(QUAD_POSE_TOPIC, &EstimatorNode::quadPoseCallback, this);
  this->registerSubscriber(QUAD_VELOCITY_TOPIC, &EstimatorNode::quadVelocityCallback, this);
  this->registerSubscriber(TARGET_INERTIAL_TOPIC, &EstimatorNode::targetWorldCallback, this);
  this->registerLoopCallback(std::bind(&EstimatorNode::loopCallback, this));
  // clang-format on

  this->configured = true;
  return 0;
}

void EstimatorNode::initLTKF(Vec3 target_wf) {
  VecX mu(9);
  MatX R(9, 9), C(6, 9), Q(6, 6);

  // initialize landing target kalman filter
  // clang-format off

  // state estimates
  mu << target_wf(0), target_wf(1), target_wf(2),
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;

  // motion noise
  R << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // measurements
  C << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  // measurement noise
  Q << 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1000.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1000.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1000.0;
  // clang-format on

  // initialize landing target kalman filter
  this->lt_kf.init(mu, R, C, Q);
}

void EstimatorNode::resetLTKF(Vec3 target_wf) {
  this->initLTKF(target_wf);
}

void EstimatorNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
  this->quad_pose = convertMsg(msg);
}

void EstimatorNode::quadVelocityCallback(const geometry_msgs::TwistStamped &msg) {
  VecX twist;
  twist = convertMsg(msg);
  this->quad_velocity = twist.block(0, 0, 3, 1);
}

void EstimatorNode::targetWorldCallback(const geometry_msgs::Vector3 &msg) {
  bool lt_kf_reset;

  // check if estimator needs resetting
  if (mtoc(&this->target_last_updated) > this->target_lost_threshold) {
    lt_kf_reset = true;
  } else {
    lt_kf_reset = false;
  }

  // update target
  this->target_detected = true;
  this->target_wf = convertMsg(msg);
  tic(&this->target_last_updated);

  // initialize or reset estimator
  if (this->lt_kf.initialized == false || lt_kf_reset) {
    this->initLTKF(this->target_wf);
  }
}

void EstimatorNode::publishLTKFWorldEstimate(void) {
  geometry_msgs::Vector3 msg;
  Vec3 estimate;

  estimate << this->lt_kf.mu(0), this->lt_kf.mu(1), this->lt_kf.mu(2);
  buildMsg(estimate, msg);

  this->ros_pubs[LT_INERTIAL_TOPIC].publish(msg);
}

void EstimatorNode::publishLTKFLocalEstimate(void) {
  geometry_msgs::Vector3 msg;
  Vec3 position_enu;

  // transform from world to body frame
  // clang-format off
  position_enu << this->lt_kf.mu(0) - this->quad_pose.position(0),
                  this->lt_kf.mu(1) - this->quad_pose.position(1),
                  this->lt_kf.mu(2) - this->quad_pose.position(2);
  // clang-format on

  // transform from ENU to NWU
  this->target_bf(0) = position_enu(1);
  this->target_bf(1) = -position_enu(0);
  this->target_bf(2) = position_enu(2);

  // build and publish msg
  buildMsg(this->target_bf, msg);
  this->ros_pubs[LT_BODY_TOPIC].publish(msg);
}

void EstimatorNode::publishLTKFVelocityEstimate(void) {
  geometry_msgs::Vector3 msg;
  Vec3 estimate;

  estimate << this->lt_kf.mu(3), this->lt_kf.mu(4), this->lt_kf.mu(5);
  buildMsg(estimate, msg);

  this->ros_pubs[LT_VELOCITY_TOPIC].publish(msg);
}

void EstimatorNode::publishGimbalSetpointAttitudeMsg(Vec3 setpoints) {
  geometry_msgs::Vector3 msg;
  buildMsg(setpoints, msg);
  this->ros_pubs[GIMBAL_SETPOINT_ATTITUDE_TOPIC].publish(msg);
}

void EstimatorNode::trackTarget(void) {
  double dist;
  Vec3 setpoints;

  // calculate roll pitch yaw setpoints
  dist = this->target_bf.norm();
  std::cout << this->target_bf.transpose() << std::endl;
  std::cout << this->target_bf.norm() << std::endl;
  std::cout << std::endl;

  setpoints(0) = asin(this->target_bf(1) / dist);   // roll
  setpoints(1) = -asin(this->target_bf(0) / dist);  // pitch
  setpoints(2) = 0.0;                                // yaw

  this->publishGimbalSetpointAttitudeMsg(setpoints);
}

int EstimatorNode::loopCallback(void) {
  MatX A(9, 9), C(6, 9);
  VecX y(6);
  double dt;

  // pre-check
  if (this->lt_kf.initialized == false) {
    return 0;
  }

  // transition matrix - constant acceleration
  // clang-format off
  dt = (ros::Time::now() - this->ros_last_updated).toSec();
  A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2.0) / 2.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2.0) / 2.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2.0) / 2.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // clang-format on

  // check measurement
  if (this->target_detected) {
    // clang-format off
    C << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    y << this->target_wf(0),
         this->target_wf(1),
         this->target_wf(2),
         this->target_wf(0) - this->target_last_wf(0),
         this->target_wf(1) - this->target_last_wf(1),
         this->target_wf(2) - this->target_last_wf(2);

    this->target_last_wf = this->target_wf;
    // clang-format on

  } else {
    // clang-format off
    C << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // clang-format on
  }

  // estimate
  this->lt_kf.C = C;
  this->lt_kf.estimate(A, y);

  // publish
  this->publishLTKFWorldEstimate();
  this->publishLTKFLocalEstimate();
  this->publishLTKFVelocityEstimate();
  this->trackTarget();

  // reset
  this->target_detected = false;
  this->target_wf << 0.0, 0.0, 0.0;

  return 0;
}

}  // end of awesomo namespace


int main(int argc, char **argv) {
  awesomo::EstimatorNode node(argc, argv);

  if (node.configure(NODE_NAME, NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure EstimatorNode!");
    return -1;
  }
  node.loop();

  return 0;
}
