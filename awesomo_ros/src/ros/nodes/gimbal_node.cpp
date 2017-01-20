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
  this->registerPublisher<geometry_msgs::Vector3>(CAMERA_RPY_TOPIC, 1);
  this->registerPublisher<geometry_msgs::Vector3>(FRAME_RPY_TOPIC, 1);
  this->registerPublisher<geometry_msgs::Vector3>(IMU_TOPIC, 1);
  this->registerPublisher<geometry_msgs::Vector3>(GYRO_TOPIC, 1);

  // subscribers
  this->registerSubscriber(SET_ANGLE_TOPIC, &GimbalNode::setAngleCallback, this);

  this->registerLoopCallback(std::bind(&GimbalNode::loopCallback, this));
  this->configured = true;
  return 0;
}

void GimbalNode::setAngleCallback(const geometry_msgs::Vector3 &msg) {
  // TODO update gimbal to take in yaw commands
  this->gimbal.setAngle(msg.x, msg.y);
}

int GimbalNode::loopCallback(void) {
   int retval;
   geometry_msgs::Vector3 cam_angles_msg;
   geometry_msgs::Vector3 frame_angles_msg;
   geometry_msgs::Vector3 imu_msg;
   geometry_msgs::Vector3 gyro_msg;

   retval = this->gimbal.updateGimbalStates();
   if (retval != 0) {
     // return -1;
   }

   cam_angles_msg.x =  this->gimbal.camera_angles(0);
   cam_angles_msg.y =  this->gimbal.camera_angles(1);
   cam_angles_msg.z =  this->gimbal.camera_angles(2);

   frame_angles_msg.x =  this->gimbal.frame_angles(0);
   frame_angles_msg.y =  this->gimbal.frame_angles(1);
   frame_angles_msg.z =  this->gimbal.frame_angles(2);

   imu_msg.x = this->gimbal.imu_accel(0);
   imu_msg.y = this->gimbal.imu_accel(1);
   imu_msg.z = this->gimbal.imu_accel(2);

   gyro_msg.x = this->gimbal.imu_gyro(0);
   gyro_msg.y = this->gimbal.imu_gyro(1);
   gyro_msg.z = this->gimbal.imu_gyro(2);

   this->ros_pubs[CAMERA_RPY_TOPIC].publish(cam_angles_msg);
   this->ros_pubs[FRAME_RPY_TOPIC].publish(frame_angles_msg);
   this->ros_pubs[IMU_TOPIC].publish(imu_msg);
   this->ros_pubs[GYRO_TOPIC].publish(gyro_msg);

   return 0;
 }

}  // end of awesomo namespace

int main(int argc, char **argv) {
  awesomo::GimbalNode node(argc, argv);

  if (node.configure(GIMBAL_NODE_NAME, GIMBAL_NODE_RATE) != 0) {
    ROS_ERROR("Failed to configure GimbalNode!");
    return -1;
  }

  node.loop();

  return 0;
}
