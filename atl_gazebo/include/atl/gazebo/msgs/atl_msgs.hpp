#ifndef atl_GAZEBO_MSGS_HPP
#define atl_GAZEBO_MSGS_HPP

#include <boost/shared_ptr.hpp>
#include <gazebo/msgs/msgs.hh>

#include "atl/gazebo/msgs/attitude_setpoint.pb.h"
#include "atl/gazebo/msgs/model_pose.pb.h"
#include "atl/gazebo/msgs/position_setpoint.pb.h"
#include "atl/gazebo/msgs/rpy_pose.pb.h"
#include "atl/gazebo/msgs/velocity_setpoint.pb.h"

#define MSG_SHARED_PTR(X, Y) typedef const boost::shared_ptr<const X> Y

MSG_SHARED_PTR(atl_msgs::msgs::AttitudeSetpoint, AttitudeSetpointPtr);
MSG_SHARED_PTR(atl_msgs::msgs::ModelPose, ModelPosePtr);
MSG_SHARED_PTR(atl_msgs::msgs::PositionSetpoint, PositionSetpointPtr);
MSG_SHARED_PTR(atl_msgs::msgs::RPYPose, RPYPosePtr);
MSG_SHARED_PTR(atl_msgs::msgs::VelocitySetpoint, VelocitySetpointPtr);

#endif
