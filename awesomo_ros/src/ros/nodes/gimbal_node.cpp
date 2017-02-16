#include "awesomo_ros/nodes/gimbal_node.hpp"

namespace awesomo {

int GimbalNode::configure(std::string node_name, int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // gimbal
  this->ros_nh->getParam("/quad_frame", this->quad_frame);
  this->ros_nh->getParam("/gimbal_config", config_file);
  if (this->gimbal.configure(config_file) != 0) {
    ROS_ERROR("Failed to configure Gimbal!");
    return -2;
  };

  // register publisher and subscribers
  // clang-format off
  this->registerPublisher<sensor_msgs::Imu>(CAMERA_IMU_TOPIC);
  this->registerPublisher<geometry_msgs::Quaternion>(FRAME_ORIENTATION_TOPIC);
  this->registerPublisher<geometry_msgs::Quaternion>(JOINT_ORIENTATION_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(POSITION_TOPIC);

  this->registerSubscriber(QUAD_POSE_TOPIC, &GimbalNode::quadPoseCallback, this);
  this->registerSubscriber(TRACK_TOPIC, &GimbalNode::trackTargetCallback, this);
  this->registerSubscriber(SETPOINT_TOPIC, &GimbalNode::setAttitudeCallback, this);
  this->registerShutdown(SHUTDOWN_TOPIC);
  // clang-format on

  // register loop callback
  this->registerLoopCallback(std::bind(&GimbalNode::loopCallback, this));

  // intialize setpoints
  this->gimbal.setAngle(0.0, -0.9);
  this->configured = true;
  return 0;
}

GimbalNode::~GimbalNode(void) {
  this->gimbal.off();
}

void GimbalNode::setAttitudeCallback(const geometry_msgs::Vector3 &msg) {
  // TODO update gimbal to take in yaw commands
  this->set_points << msg.x, msg.y, msg.z;
  // this->gimbal.setAngle(msg.x, msg.y);
  // this->gimbal.setAngle(msg.x, msg.y);
  // usleep(7000); //sleep for .7 milliseconds
}

void GimbalNode::trackTargetCallback(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d target_cf;
  target_cf << msg.x, msg.y, msg.z;
  this->gimbal.trackTarget(target_cf);
}

void GimbalNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
  geometry_msgs::Vector3 ros_msg;

  if (this->quad_frame == "NWU") {
    // transform from NWU to ENU
    ros_msg.x = -msg.pose.position.y;
    ros_msg.y = msg.pose.position.x;
    ros_msg.z = msg.pose.position.z;
  } else if (this->quad_frame == "NED") {
    // transform from NED to ENU
    ros_msg.x = msg.pose.position.y;
    ros_msg.y = msg.pose.position.x;
    ros_msg.z = msg.pose.position.z;
  }

  this->ros_pubs[POSITION_TOPIC].publish(ros_msg);
}

int GimbalNode::loopCallback(void) {
   int retval;
   sensor_msgs::Imu cam_imu_msg;
   geometry_msgs::Quaternion frame_angles_msg;
   geometry_msgs::Quaternion cam_angles_msg;

   Vec3 camera_euler;
   Vec3 frame_euler;
   Quaternion camera_orientation;
   Quaternion frame_orientation;

   this->gimbal.setAngle(this->set_points(0), this->set_points(1));
   usleep(12000); // sleep for 10 microseconds to help avoid dropping frames
   retval = this->gimbal.updateGimbalStates();
   if (retval != 0) {
     return 0;
     // return -1; // some frame will drop, we need to live with this
   }

   // clang-format off
   camera_euler << this->gimbal.camera_angles(0),
                   this->gimbal.camera_angles(1),
                   this->gimbal.camera_angles(2);
   // clang-format on

   // clang-format off
   frame_euler << this->gimbal.frame_angles(0),
                  this->gimbal.frame_angles(1),
                  this->gimbal.frame_angles(2);
   // clang-format on

   // convert angles to quaterions
   euler2quat(camera_euler, 321, camera_orientation);
   euler2quat(frame_euler, 321, frame_orientation);

   cam_imu_msg.header.seq = this->ros_seq;
   cam_imu_msg.header.stamp = ros::Time::now();
   cam_imu_msg.header.frame_id = "gimbal_joint";

   cam_imu_msg.orientation.x = camera_orientation.x();
   cam_imu_msg.orientation.y = camera_orientation.y();
   cam_imu_msg.orientation.z = camera_orientation.z();
   cam_imu_msg.orientation.w = camera_orientation.w();

   cam_imu_msg.angular_velocity.x = this->gimbal.imu_gyro(0);
   cam_imu_msg.angular_velocity.y = this->gimbal.imu_gyro(1);
   cam_imu_msg.angular_velocity.z = this->gimbal.imu_gyro(2);

   cam_imu_msg.linear_acceleration.x = this->gimbal.imu_accel(0);
   cam_imu_msg.linear_acceleration.y = this->gimbal.imu_accel(1);
   cam_imu_msg.linear_acceleration.z = this->gimbal.imu_accel(2);

   buildMsg(frame_orientation, frame_angles_msg);
   buildMsg(camera_orientation, cam_angles_msg);

   this->ros_pubs[CAMERA_IMU_TOPIC].publish(cam_imu_msg);
   this->ros_pubs[FRAME_ORIENTATION_TOPIC].publish(frame_angles_msg);
   this->ros_pubs[JOINT_ORIENTATION_TOPIC].publish(cam_angles_msg);
   // usleep(10000); // sleep for 10 microseconds to help avoid dropping frames

   return 0;
 }

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::GimbalNode, NODE_NAME, NODE_RATE);
