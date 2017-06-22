#ifndef WAVESIM_GAZEBO_HPP
#define WAVESIM_GAZEBO_HPP

#include "clients/camera_gclient.hpp"
#include "clients/gimbal_gclient.hpp"
#include "clients/quadrotor_gclient.hpp"
#include "clients/world_gclient.hpp"

#include "msgs/attitude_setpoint.pb.h"
#include "msgs/header.pb.h"
#include "msgs/position_setpoint.pb.h"
#include "msgs/prototype_msgs.hpp"
#include "msgs/rpy_pose.pb.h"
#include "msgs/time.pb.h"
#include "msgs/vector3d.pb.h"

#include "plugins/camera_gplugin.hpp"
#include "plugins/gimbal_gplugin.hpp"
#include "plugins/landing_zone_gplugin.hpp"
#include "plugins/quadrotor_gplugin.hpp"
#include "plugins/world_gplugin.hpp"

#include "gazebo_node.hpp"

#endif
