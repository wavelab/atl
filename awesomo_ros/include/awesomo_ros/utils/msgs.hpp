#ifndef __AWESOMO_ROS_UTILS_MSGS_HPP__
#define __AWESOMO_ROS_UTILS_MSGS_HPP__

#include <std_msgs/Float64.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <awesomo_core/awesomo_core.hpp>

#include <awesomo_msgs/AprilTagPose.h>
#include <awesomo_msgs/PCtrlStats.h>
#include <awesomo_msgs/PCtrlSettings.h>
#include <awesomo_msgs/TCtrlStats.h>
#include <awesomo_msgs/TCtrlSettings.h>
#include <awesomo_msgs/VCtrlStats.h>
#include <awesomo_msgs/VCtrlSettings.h>


namespace awesomo {

void buildMsg(Vec3 vec, geometry_msgs::Vector3 &msg);
void buildMsg(int seq,
              ros::Time time,
              AttitudeCommand att_cmd,
              geometry_msgs::PoseStamped &msg,
              std_msgs::Float64 &thr_msg);
void buildMsg(TagPose tag, awesomo_msgs::AprilTagPose &msg);
void buildMsg(TagPose tag, geometry_msgs::Vector3 &msg);
void buildMsg(PositionController pc, awesomo_msgs::PCtrlStats &msg);
void buildMsg(PositionController pc, awesomo_msgs::PCtrlSettings &msg);
void buildMsg(TrackingController tc, awesomo_msgs::TCtrlStats &msg);
void buildMsg(TrackingController tc, awesomo_msgs::TCtrlSettings &msg);

Vec3 convertMsg(geometry_msgs::Vector3 msg);
Pose convertMsg(geometry_msgs::PoseStamped msg);
VecX convertMsg(geometry_msgs::TwistStamped msg);
TagPose convertMsg(awesomo_msgs::AprilTagPose msg);

}  // end of awesomo namespace
#endif
