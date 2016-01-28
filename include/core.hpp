#ifndef __CORE_H__
#define __CORE_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// FUNCTIONS
int core(int argc, char **argv);


#endif
