#ifndef WAVESIM_GAZEBO_MSGS_HPP
#define WAVESIM_GAZEBO_MSGS_HPP

#include <boost/shared_ptr.hpp>
#include <gazebo/msgs/msgs.hh>

#include "wavesim_gazebo/msgs/attitude_setpoint.pb.h"
#include "wavesim_gazebo/msgs/model_pose.pb.h"
#include "wavesim_gazebo/msgs/position_setpoint.pb.h"
#include "wavesim_gazebo/msgs/rpy_pose.pb.h"
#include "wavesim_gazebo/msgs/velocity_setpoint.pb.h"

#define MSG_SHARED_PTR(X, Y) typedef const boost::shared_ptr<const X> Y

MSG_SHARED_PTR(wavesim_msgs::msgs::AttitudeSetpoint, AttitudeSetpointPtr);
MSG_SHARED_PTR(wavesim_msgs::msgs::ModelPose, ModelPosePtr);
MSG_SHARED_PTR(wavesim_msgs::msgs::PositionSetpoint, PositionSetpointPtr);
MSG_SHARED_PTR(wavesim_msgs::msgs::RPYPose, RPYPosePtr);
MSG_SHARED_PTR(wavesim_msgs::msgs::VelocitySetpoint, VelocitySetpointPtr);

#endif
