#ifndef ATL_ROS_UTILS_MSGS_HPP
#define ATL_ROS_UTILS_MSGS_HPP

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

#include <atl/atl_core.hpp>

#include <atl_msgs/AprilTagPose.h>
#include <atl_msgs/LCtrlSettings.h>
#include <atl_msgs/ModelPose.h>
#include <atl_msgs/PCtrlSettings.h>
#include <atl_msgs/TCtrlSettings.h>
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
void buildMsg(TagPose tag, atl_msgs::AprilTagPose &msg);
void buildMsg(TagPose tag, geometry_msgs::Vector3 &msg);
void buildMsg(PositionController pc, atl_msgs::PCtrlSettings &msg);
void buildMsg(TrackingController tc, atl_msgs::TCtrlSettings &msg);
void buildMsg(LandingController tc, atl_msgs::LCtrlSettings &msg);

void convertMsg(std_msgs::UInt8 msg, uint8_t &x);
void convertMsg(std_msgs::Bool msg, bool &b);
void convertMsg(std_msgs::String msg, std::string &s);
void convertMsg(std_msgs::Float64 msg, double &d);
void convertMsg(geometry_msgs::Vector3 msg, Vec3 &v);
void convertMsg(geometry_msgs::Vector3Stamped msg, Vec3 &v);
void convertMsg(geometry_msgs::Point msg, Vec3 &v);
void convertMsg(geometry_msgs::Quaternion msg, Quaternion &q);
void convertMsg(geometry_msgs::QuaternionStamped msg, Quaternion &q);
void convertMsg(geometry_msgs::PoseStamped msg, Pose &p);
void convertMsg(geometry_msgs::TwistStamped msg, VecX &v);
void convertMsg(atl_msgs::AprilTagPose msg, TagPose &p);
void convertMsg(atl_msgs::PCtrlSettings msg, PositionController &pc);
void convertMsg(atl_msgs::TCtrlSettings msg, TrackingController &tc);
void convertMsg(atl_msgs::LCtrlSettings msg, LandingController &lc);

} // namespace atl
#endif
