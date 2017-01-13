#include "awesomo_ros/nodes/estimate_node.hpp"

namespace awesomo {

int EstimateNode::configure(std::string node_name, int hz) {
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
  this->registerSubscriber(QUAD_POSE_TOPIC, &EstimateNode::quadPoseCallback, this);
  this->registerSubscriber(GIMBAL_TARGET_INERTIAL_TOPIC, &EstimateNode::gimbalTargetWorldCallback, this);
  this->registerLoopCallback(std::bind(&EstimateNode::loopCallback, this));
  // clang-format on

  this->configured = true;
  return 0;
}

void EstimateNode::initLTKF(Vec3 target_wpf) {
  VecX mu(9);
  MatX R(9, 9), C(6, 9), Q(6, 6);

  // initialize landing target kalman filter
  // clang-format off

  // state estimates
  mu << target_wpf(0), target_wpf(1), target_wpf(2),
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;

  // motion noise
  R << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
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
  Q << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // initialize landing target kalman filter
  this->lt_kf.init(mu, R, C, Q);
}

void EstimateNode::resetLTKF(Vec3 target_wpf) {
  this->initLTKF(target_wpf);
}

void EstimateNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
  this->quad_pose = convertPoseStampedMsg2Pose(msg);
}

void EstimateNode::gimbalTargetWorldCallback(
  const geometry_msgs::Vector3 &msg) {
  bool lt_kf_reset;

  // check if estimator needs resetting
  if (mtoc(&this->target_last_updated) > this->target_lost_threshold) {
    lt_kf_reset = true;
  } else {
    lt_kf_reset = false;
  }

  // update target
  this->target_detected = true;
  this->target_wpf = convertVector3Msg2Vec3(msg);
  tic(&this->target_last_updated);

  // initialize or reset estimator
  if (this->lt_kf.initialized == false || lt_kf_reset) {
    this->initLTKF(this->target_wpf);
  }
}

void EstimateNode::publishLTKFWorldEstimate(void) {
  geometry_msgs::Vector3 msg;
  Vec3 estimate;

  estimate << this->lt_kf.mu(0), this->lt_kf.mu(1), this->lt_kf.mu(2);
  buildVector3Msg(estimate, msg);

  this->ros_pubs[LT_INERTIAL_TOPIC].publish(msg);
}

void EstimateNode::publishLTKFLocalEstimate(void) {
  geometry_msgs::Vector3 msg;
  Vec3 estimate_enu, estimate_nwu;

  // transform from world to body frame
  // clang-format off
  estimate_enu << this->lt_kf.mu(0) - this->quad_pose.position(0),
                  this->lt_kf.mu(1) - this->quad_pose.position(1),
                  this->lt_kf.mu(2) - this->quad_pose.position(2);
  // clang-format on

  // transform from ENU to NWU
  estimate_nwu(0) = estimate_enu(1);
  estimate_nwu(1) = -estimate_enu(0);
  estimate_nwu(2) = estimate_enu(2);

  buildVector3Msg(estimate_nwu, msg);

  this->ros_pubs[LT_BODY_TOPIC].publish(msg);
}

void EstimateNode::publishLTKFVelocityEstimate(void) {
  geometry_msgs::Vector3 msg;
  Vec3 estimate;

  estimate << this->lt_kf.mu(3), this->lt_kf.mu(4), this->lt_kf.mu(5);
  buildVector3Msg(estimate, msg);

  this->ros_pubs[LT_VELOCITY_TOPIC].publish(msg);
}

int EstimateNode::loopCallback(void) {
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

    y << this->target_wpf(0),
         this->target_wpf(1),
         this->target_wpf(2),
         this->target_wpf(0) - this->target_last_wpf(0),
         this->target_wpf(1) - this->target_last_wpf(1),
         this->target_wpf(2) - this->target_last_wpf(2);

    this->target_last_wpf = this->target_wpf;
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
  this->target_detected = false;
  this->target_wpf << 0.0, 0.0, 0.0;

  // publish
  this->publishLTKFWorldEstimate();
  this->publishLTKFLocalEstimate();
  this->publishLTKFVelocityEstimate();

  return 0;
}

}  // end of awesomo namespace


int main(int argc, char **argv) {
  awesomo::EstimateNode node(argc, argv);

  if (node.configure(NODE_NAME, NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure EstimateNode!");
    return -1;
  }
  node.loop();

  return 0;
}
