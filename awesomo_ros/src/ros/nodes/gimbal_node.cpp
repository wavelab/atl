#include "awesomo_ros/nodes/gimbal_node.hpp"

namespace awesomo {

int GimbalNode::configure(std::string node_name, int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // gimbal
  this->ros_nh->getParam("/gimbal_config", config_file);
  if (this->gimbal.configure(config_file) != 0) {
    ROS_ERROR("Failed to configure Gimbal!");
    return -2;
  };

  // register publisher and subscribers
  // clang-format off
  this->registerPublisher<sensor_msgs::Imu>(CAMERA_IMU_TOPIC);
  this->registerPublisher<geometry_msgs::Quaternion>(FRAME_RPY_TOPIC);
  this->registerSubscriber(SET_ATTITUDE_TOPIC, &GimbalNode::setAttitudeCallback, this);
  this->registerShutdown(SHUTDOWN_TOPIC);
  // clang-format on

  this->registerLoopCallback(std::bind(&GimbalNode::loopCallback, this));
  this->seq = 1;
  this->configured = true;
  return 0;
}

GimbalNode::~GimbalNode(void) {
  this->gimbal.off();
}

void GimbalNode::setAttitudeCallback(const geometry_msgs::Vector3 &msg) {
  // TODO update gimbal to take in yaw commands
  this->gimbal.setAngle(msg.x, msg.y);
}

int GimbalNode::loopCallback(void) {
   int retval;
   sensor_msgs::Imu cam_imu_msg;
   geometry_msgs::Quaternion frame_angles_msg;

   Vec3 cam_angles_euler;
   Vec3 frame_angles_euler;
   Quaternion cam_quat;
   Quaternion frame_quat;


   retval = this->gimbal.updateGimbalStates();
   if (retval != 0) {
       return 0;
     // return -1; // some frame will drop, we need to live with this
   }

   cam_angles_euler <<
     this->gimbal.camera_angles(0),
     this->gimbal.camera_angles(1),
     this->gimbal.camera_angles(2);

   frame_angles_euler <<
     this->gimbal.frame_angles(0),
     this->gimbal.frame_angles(1),
     this->gimbal.frame_angles(2);

   // convert angles to quaterions
   euler2quat(cam_angles_euler, 123, cam_quat);
   euler2quat(frame_angles_euler, 123, frame_quat);

   cam_imu_msg.orientation.x = cam_quat.x();
   cam_imu_msg.orientation.y = cam_quat.y();
   cam_imu_msg.orientation.z = cam_quat.z();
   cam_imu_msg.orientation.w = cam_quat.w();

   cam_imu_msg.angular_velocity.x = this->gimbal.imu_gyro(0);
   cam_imu_msg.angular_velocity.y = this->gimbal.imu_gyro(1);
   cam_imu_msg.angular_velocity.z = this->gimbal.imu_gyro(2);

   cam_imu_msg.linear_acceleration.x = this->gimbal.imu_accel(0);
   cam_imu_msg.linear_acceleration.y = this->gimbal.imu_accel(1);
   cam_imu_msg.linear_acceleration.z = this->gimbal.imu_accel(2);

   cam_imu_msg.header.seq = this->seq;
   cam_imu_msg.header.stamp = ros::Time::now();
   cam_imu_msg.header.frame_id = "gimbal_joint";

   this->seq += 1;

   frame_angles_msg.x = frame_quat.x();
   frame_angles_msg.y = frame_quat.y();
   frame_angles_msg.z = frame_quat.z();
   frame_angles_msg.w = frame_quat.w();

   this->ros_pubs[CAMERA_IMU_TOPIC].publish(cam_imu_msg);
   this->ros_pubs[FRAME_RPY_TOPIC].publish(frame_angles_msg);

   return 0;
 }

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::GimbalNode, NODE_NAME, NODE_RATE);
