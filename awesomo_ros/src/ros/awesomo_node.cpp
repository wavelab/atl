#include "awesomo_ros/awesomo_node.hpp"


namespace awesomo {

AwesomoNode::AwesomoNode(void) {
  for (int i = 0; i < 16; i++) {
    this->rc_in[i] = 0.0f;
  }
}

int AwesomoNode::configure(std::map<std::string, std::string> configs) {
  std::string config_path;

  this->quad = Quadrotor(configs);

  // // wait till connected to FCU
  // this->waitForConnection();
  //
  // // initialize clients to services
  // this->mode_client =
  //   this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
  // this->arming_client =
  //   this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);
  //
  // // initialize subscribers
  // this->subscribeToPose();
  // this->subscribeToVelocity();
  // this->subscribeToRadioIn();
  // this->subscribeToAtim();
  //
  // // initialize publishers
  // this->position_publisher =
  //   this->node.advertise<geometry_msgs::PoseStamped>(POSITION_TOPIC, 50);
  // this->attitude_publisher =
  //   this->node.advertise<geometry_msgs::PoseStamped>(ATTITUDE_TOPIC, 50);
  // this->attitude_rpy_publisher =
  //   this->node.advertise<geometry_msgs::Vector3>(ATTITUDE_RPY_TOPIC, 50);
  // this->throttle_publisher =
  //   this->node.advertise<std_msgs::Float64>(THROTTLE_TOPIC, 50);
  // this->position_controller_stats_publisher =
  //   this->node.advertise<awesomo_msgs::PositionControllerStats>(
  //     POSITION_CONTROLLER_TOPIC, 50);
  // this->kf_estimator_stats_publisher =
  //   this->node.advertise<awesomo_msgs::KFStats>(KF_ESTIMATION_TOPIC, 50);
  // this->kf_estimator_stats_plotting_publisher =
  //   this->node.advertise<awesomo_msgs::KFPlotting>(
  //     KF_ESTIMATION_PLOTTING_TOPIC, 50);
}

// void Awesomo::poseCallback(const geometry_msgs::PoseStamped &msg) {
//   Eigen::Quaterniond quat;
//   Eigen::Vector3d position;
//
//   quat = Eigen::Quaterniond(msg.pose.orientation.w,
//                             msg.pose.orientation.x,
//                             msg.pose.orientation.y,
//                             msg.pose.orientation.z);
//
//   position << msg.pose.position.x, msg.pose.position.y,
//     msg.pose.position.z + this->quad->height_offset;
//
//   // set height offset
//   // NOTE: this assumes the quadrotor starts off from the ground
//   if (this->quad->height_offset_initialized == false) {
//     this->quad->height_offset = 0.0 - msg.pose.position.z;
//     this->quad->height_offset_initialized = true;
//     printf("ASSUMING QUADROTOR IS ON THE GROUND!!!\n");
//     printf("height_offset: %f\n", this->quad->height_offset);
//   }
//
//   // ENU coordinates
//   this->world_pose = Pose(quat, position);
//
//   this->world_imu_tf.setOrigin(tf::Vector3(this->world_pose.position(0),
//                                            this->world_pose.position(1),
//                                            this->world_pose.position(2)));
//
//   // for display in rviz, serves no other function
//   this->world_imu_tf.setRotation(tf::Quaternion(msg.pose.orientation.w,
//                                                 msg.pose.orientation.x,
//                                                 msg.pose.orientation.y,
//                                                 msg.pose.orientation.z));
//
//   this->world_imu_tf_broadcaster.sendTransform(
//     tf::StampedTransform(world_imu_tf, ros::Time::now(), "world",
//     "pixhawk"));
// }
//
// void Awesomo::velocityCallback(const geometry_msgs::TwistStamped &msg) {
//   this->velocity.linear_x = msg.twist.linear.x;
//   this->velocity.linear_y = msg.twist.linear.y;
//   this->velocity.linear_z = msg.twist.linear.z;
//
//   this->velocity.angular_y = msg.twist.angular.y;
//   this->velocity.angular_z = msg.twist.angular.z;
// }
//
// void Awesomo::mocapCallback(const geometry_msgs::PoseStamped &msg) {
//   // mocap position
//   Eigen::Quaterniond quat;
//   Eigen::Vector3d position;
//
//   quat = Eigen::Quaterniond(msg.pose.orientation.w,
//                             msg.pose.orientation.x,
//                             msg.pose.orientation.y,
//                             msg.pose.orientation.z);
//
//   position << msg.pose.position.x, msg.pose.position.y,
//   msg.pose.position.z;
//
//   this->mocap_pose = Pose(quat, position);
// }
//
// void Awesomo::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
//   state = *msg;
// }
//
// void Awesomo::radioCallback(const mavros_msgs::RCIn &msg) {
//   for (int i = 0; i < 16; i++) {
//     this->rc_in[i] = msg.channels[i];
//   }
// }
//
// void Awesomo::atimCallback(const atim::AtimPoseStamped &msg) {
//   this->landing_zone.detected = msg.tag_detected;
//   this->landing_zone.position << msg.pose.position.x, msg.pose.position.y,
//     msg.pose.position.z;
// }
//
// void Awesomo::gpsCallback(
//   const geometry_msgs::PoseWithCovarianceStamped &msg) {
//   Eigen::Quaterniond quat;
//   Eigen::Vector3d position;
//
//   quat = Eigen::Quaterniond(msg.pose.pose.orientation.w,
//                             msg.pose.pose.orientation.x,
//                             msg.pose.pose.orientation.y,
//                             msg.pose.pose.orientation.z);
//
//   position << msg.pose.pose.position.x, msg.pose.pose.position.y,
//     msg.pose.pose.position.z;
//
//   this->gps_pose = Pose(quat, position);
// }
//
// void Awesomo::waitForConnection(void) {
//   ROS_INFO("waiting for FCU ...");
//   while (ros::ok() && this->state.connected) {
//     ros::spinOnce();
//     sleep(1);
//   }
//   ROS_INFO("connected to FCU!");
// }
//
// int Awesomo::arm(void) {
//   mavros_msgs::CommandBool arm_req;
//
//   // setup
//   ROS_INFO("arming awesomo ...");
//   arm_req.request.value = true;
//
//   // arm
//   if (this->arming_client.call(arm_req)) {
//     ROS_INFO("awesomo armed!");
//   } else {
//     ROS_ERROR("failed to arm awesomo!");
//   }
//
//   return 0;
// }
//
// int Awesomo::disarm(void) {
//   mavros_msgs::CommandBool arm_req;
//
//   // setup
//   ROS_INFO("disarming awesomo ...");
//   arm_req.request.value = false;
//
//   // arm
//   if (this->arming_client.call(arm_req)) {
//     ROS_INFO("awesomo disarmed!");
//   } else {
//     ROS_ERROR("failed to disarm awesomo!");
//   }
//
//   return 0;
// }
//
// int Awesomo::setOffboardModeOn(void) {
//   mavros_msgs::SetMode mode;
//
//   // setup
//   mode.request.custom_mode = "OFFBOARD";
//
//   if (mode_client.call(mode) && mode.response.success) {
//     ROS_INFO("Offboard enabled");
//     return 0;
//   } else {
//     return -1;
//   }
// }
//
// void Awesomo::subscribeToPose(void) {
//   ROS_INFO("subcribing to [POSE]");
//   this->world_pose_subscriber =
//     this->node.subscribe(POSE_TOPIC, 10, &Awesomo::poseCallback, this);
// }
//
//
// void Awesomo::subscribeToVelocity(void) {
//   ROS_INFO("subcribing to [VELOCITY]");
//   this->velocity_subscriber = this->node.subscribe(
//     VELOCITY_TOPIC, 10, &Awesomo::velocityCallback, this);
// }
//
// void Awesomo::subscribeToMocap(void) {
//   ROS_INFO("subcribing to [MOCAP]");
//   this->mocap_subscriber =
//     this->node.subscribe(MOCAP_TOPIC, 10, &Awesomo::mocapCallback, this);
// }
//
// void Awesomo::subscribeToRadioIn(void) {
//   ROS_INFO("subscribing to [RADIO_IN]");
//   this->radio_subscriber =
//     this->node.subscribe(RADIO_TOPIC, 10, &Awesomo::radioCallback, this);
// }
//
// void Awesomo::subscribeToAtim(void) {
//   ROS_INFO("subcribing to [ATIM]");
//   this->landing_subscriber =
//     this->node.subscribe(ATIM_POSE_TOPIC, 10, &Awesomo::atimCallback,
//     this);
// }
//
// void Awesomo::subscribeToGPS(void) {
//   ROS_INFO("subcribing to [GPS UTM]");
//   this->gps_subscriber =
//     this->node.subscribe(GPS_TOPIC, 10, &Awesomo::gpsCallback, this);
// }
//
// void Awesomo::publishHoverCommand(int seq, ros::Time time) {
//   geometry_msgs::PoseStamped hover_cmd;
//   float adjusted_height;
//
//   // setup
//   adjusted_height = this->quad->hover_height + this->quad->height_offset;
//
//   // msg header
//   hover_cmd.header.seq = seq;
//   hover_cmd.header.stamp = time;
//   hover_cmd.header.frame_id = "awesomo_hover_position";
//
//   // set pose
//   hover_cmd.pose.position.x = this->hover_point.position(0);
//   hover_cmd.pose.position.y = this->hover_point.position(1);
//   hover_cmd.pose.position.z = adjusted_height;
//   hover_cmd.pose.orientation.w = this->hover_point.q.w();
//   hover_cmd.pose.orientation.x = this->hover_point.q.x();
//   hover_cmd.pose.orientation.y = this->hover_point.q.y();
//   hover_cmd.pose.orientation.z = this->hover_point.q.z();
//
//   this->position_publisher.publish(hover_cmd);
// }

// int Awesomo::run(geometry_msgs::PoseStamped &msg,
//                  int seq,
//                  ros::Time last_request) {
//   float dt;
//   int retval;
//   // calculate attitude from position controller
//   dt = (ros::Time::now() - last_request).toSec();
//
//   // run mission
//   retval = this->quad->runMission(this->world_pose, this->landing_zone,
//   dt);
//   if (retval == MISSION_ACCOMPLISHED) {
//     this->disarm();
//     return 0;
//
//   } else if (retval == DISCOVER_MODE) {
//     this->quad->resetPositionController();
//     this->publishHoverCommand(seq, ros::Time::now());
//     return 1;
//
//   } else {
//     this->hover_point = this->world_pose;  // keep track of hover point
//     this->publishPositionControllerMessage(msg, seq, ros::Time::now());
//     this->publishPositionControllerStats(seq, ros::Time::now());
//     this->publishKFStatsForPlotting(seq, ros::Time::now());
//     return 1;
//   }
// }

}  // end of awesomo namespace

int main(int argc, char **argv) {
  // setup
  ros::init(argc, argv, "awesomo");
  ros::NodeHandle node_handle;
  ros::Rate rate(100.0);
  ros::Time last_request;
  geometry_msgs::PoseStamped msg;

  float dt;
  int seq;
  awesomo::AwesomoNode awesomo;
  std::string quadrotor_config;
  std::string position_controller_config;
  std::map<std::string, std::string> configs;
  std_msgs::Float64 throttle;

  // get configuration paths
  node_handle.getParam("/quadrotor", quadrotor_config);
  node_handle.getParam("/position_controller", position_controller_config);
  configs["quadrotor"] = quadrotor_config;
  configs["position_controller"] = position_controller_config;

  // setup awesomo
  seq = 1;
  ROS_INFO("running ...");
  awesomo.configure(configs);
  last_request = ros::Time::now();

  // #ifndef YAW_CONTROL_ON
  //   printf("YAW CONTROL IS OFF!\n");
  // #else
  //   printf("YAW CONTROL IS ON!\n");
  // #endif

  while (ros::ok()) {
    // if (awesomo->run(msg, seq, last_request) == 0) {
    //   return 0;
    // }
    // end
    seq++;
    last_request = ros::Time::now();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
