#ifndef __AWESOMO_ROS_UTILS_MSGS_HPP__
#define __AWESOMO_ROS_UTILS_MSGS_HPP__

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
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

void buildMsg(bool b, std_msgs::Bool &msg);
void buildMsg(std::string s, std_msgs::String &msg);
void buildMsg(double d, std_msgs::Float64 &msg);
void buildMsg(Vec3 vec, geometry_msgs::Vector3 &msg);
void buildMsg(Vec3 vec, geometry_msgs::Point &msg);
void buildMsg(Quaternion q, geometry_msgs::Quaternion &msg);
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

bool convertMsg(std_msgs::Bool msg);
std::string convertMsg(std_msgs::String msg);
double convertMsg(std_msgs::Float64 msg);
Vec3 convertMsg(geometry_msgs::Vector3 msg);
Vec3 convertMsg(geometry_msgs::Point msg);
Quaternion convertMsg(geometry_msgs::Quaternion msg);
Pose convertMsg(geometry_msgs::PoseStamped msg);
VecX convertMsg(geometry_msgs::TwistStamped msg);
TagPose convertMsg(awesomo_msgs::AprilTagPose msg);

}  // end of awesomo namespace
#endif
