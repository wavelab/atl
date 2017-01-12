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
  this->registerPublisher<geometry_msgs::Vector3>(LT_WORLD_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(LT_LOCAL_TOPIC);
  this->registerSubscriber(GIMBAL_TARGET_TOPIC, &EstimateNode::gimbalTargetCallback, this);
  this->registerSubscriber(LT_INIT_TOPIC, &EstimateNode::initLTKFCallback, this);
  // clang-format on

  this->configured = true;
  return 0;
}

void EstimateNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
  this->quad_pose = convertPoseStampedMsg2Pose(msg);
}

void EstimateNode::gimbalTargetCallback(const geometry_msgs::Vector3 &msg) {
  this->target_bpf = convertVector3Msg2Vec3(msg);
}

void EstimateNode::initLTKFCallback(const std_msgs::Bool &msg) {
  VecX mu(9);
  MatX R(9, 9), C(3, 9), Q(3, 3);

  // pre-check
  if (msg.data == false) {
    return;
  }

  // initialize landing target kalman filter
  // clang-format off
  mu << this->quad_pose.position(0),
        this->quad_pose.position(1),
        this->quad_pose.position(2),
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;

  R << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  C << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Q << 20.0, 0.0, 0.0,
       0.0, 20.0, 0.0,
       0.0, 0.0, 20.0;
  // clang-format on

  // initialize landing target kalman filter
  this->lt_kf.init(mu, R, C, Q);
}

void EstimateNode::publishLTKFWorldEstimate(void) {
  geometry_msgs::Vector3 msg;
  Vec3 estimate;

  estimate << this->lt_kf.mu(0), this->lt_kf.mu(1), this->lt_kf.mu(2);
  buildVector3Msg(etimate, msg);

  this->ros_pubs[LT_WORLD_TOPIC].publish(msg);
}

void EstimateNode::publishLTKFLocalEstimate(void) {
  geometry_msgs::Vector3 msg;
  Vec3 estimate;

  // clang-format off
  estimate << this->quad_pose.position(0) - this->lt_kf.mu(0),
              this->quad_pose.position(1) - this->lt_kf.mu(1),
              this->quad_pose.position(2) - this->lt_kf.mu(2);
  // clang-format on
  buildVector3Msg(estimate, msg);

  this->ros_pubs[LT_LOCAL_TOPIC].publish(msg);
}

int EstimateNode::loopCallback(void) {
  MatX A(9, 9), C(3, 9);
  Vec3 target_wpf;

  // transition matrix - constant acceleration
  // clang-format off
  A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2.0) / 2.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2.0) / 2.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2.0) / 2.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // check measurement
  if (this->target_detected) {
    // clang-format off
    C << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    target_wpf << 0.0, 0.0, 0.0;
    // clang-format on

  } else {
    // clang-format off
    C << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    target_wpf << 0.0, 0.0, 0.0;
    // clang-format on
  }

  // estimate
  this->lt_kf.C = C;
  this->lt_kf.estimate(A, target_wpf);

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
