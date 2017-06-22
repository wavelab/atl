#ifndef ATL_ROS_UTILS_MSGS_HPP
#define ATL_ROS_UTILS_MSGS_HPP

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <atl_core/atl_core.hpp>

#include <atl_msgs/AprilTagPose.h>
#include <atl_msgs/LCtrlSettings.h>
#include <atl_msgs/PCtrlStats.h>
#include <atl_msgs/PCtrlSettings.h>
#include <atl_msgs/TCtrlStats.h>
#include <atl_msgs/TCtrlSettings.h>
#include <atl_msgs/VCtrlStats.h>
#include <atl_msgs/VCtrlSettings.h>


namespace atl {

void buildMsg(bool b, std_msgs::Bool &msg);
void buildMsg(std::string s, std_msgs::String &msg);
void buildMsg(double d, std_msgs::Float64 &msg);
void buildMsg(Vec3 vec, geometry_msgs::Vector3 &msg);
void buildMsg(Vec3 vec, geometry_msgs::Point &msg);
void buildMsg(Quaternion q, geometry_msgs::Quaternion &msg);
void buildMsg(int seq,
              ros::Time time,
              Pose pose,
              geometry_msgs::PoseStamped &msg);
void buildMsg(int seq,
              ros::Time time,
              AttitudeCommand att_cmd,
              geometry_msgs::PoseStamped &msg,
              std_msgs::Float64 &thr_msg);
void buildMsg(TagPose tag, atl_msgs::AprilTagPose &msg);
void buildMsg(TagPose tag, geometry_msgs::Vector3 &msg);
void buildMsg(PositionController pc, atl_msgs::PCtrlStats &msg);
void buildMsg(PositionController pc, atl_msgs::PCtrlSettings &msg);
void buildMsg(TrackingController tc, atl_msgs::TCtrlStats &msg);
void buildMsg(TrackingController tc, atl_msgs::TCtrlSettings &msg);
void buildMsg(LandingController tc, atl_msgs::LCtrlSettings &msg);

int convertMsg(std_msgs::Bool msg, bool &b);
int convertMsg(std_msgs::String msg, std::string &s);
int convertMsg(std_msgs::Float64 msg, double &d);
int convertMsg(geometry_msgs::Vector3 msg, Vec3 &v);
int convertMsg(geometry_msgs::Point msg, Vec3 &v);
int convertMsg(geometry_msgs::Quaternion msg, Quaternion &q);
int convertMsg(geometry_msgs::PoseStamped msg, Pose &p);
int convertMsg(geometry_msgs::TwistStamped msg, VecX &v);
int convertMsg(atl_msgs::AprilTagPose msg, TagPose &p);
int convertMsg(atl_msgs::PCtrlSettings msg, PositionController &pc);
int convertMsg(atl_msgs::TCtrlSettings msg, TrackingController &tc);
int convertMsg(atl_msgs::LCtrlSettings msg, LandingController &lc);

}  // end of atl namespace
#endif
