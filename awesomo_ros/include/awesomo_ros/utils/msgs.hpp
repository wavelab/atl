#ifndef __AWESOMO_ROS_UTILS_MSGS_HPP__
#define __AWESOMO_ROS_UTILS_MSGS_HPP__

#include <std_msgs/Float64.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <awesomo_core/awesomo_core.hpp>

#include <awesomo_msgs/KFPlotting.h>
#include <awesomo_msgs/KFStats.h>
#include <awesomo_msgs/AprilTagPose.h>
#include <awesomo_msgs/PCtrlStats.h>
#include <awesomo_msgs/PCtrlSettings.h>


namespace awesomo {

void buildPCtrlStatsMsg(int seq,
                        ros::Time time,
                        PositionController controller,
                        awesomo_msgs::PCtrlStats &msg);
void buildAttitudeMsg(int seq,
                      ros::Time time,
                      AttitudeCommand att_cmd,
                      geometry_msgs::PoseStamped &msg,
                      std_msgs::Float64 &thr_msg);
void buildKFStatsMsg(int seq,
                     ros::Time time,
                     KalmanFilter estimator,
                     awesomo_msgs::KFStats &msg);
void buildKFPlottingMsg(int seq,
                        ros::Time time,
                        KalmanFilter estimator,
                        awesomo_msgs::KFPlotting &msg);
void buildAprilTagPoseMsg(TagPose tag, awesomo_msgs::AprilTagPose &msg);
void buildAprilTagTrackMsg(TagPose tag, geometry_msgs::Vector3 &msg);
void buildPCtrlSettingsMsg(PositionController pc,
                           awesomo_msgs::PCtrlSettings &msg);
Pose convertPoseStampedMsg2Pose(geometry_msgs::PoseStamped msg);
TagPose convertAprilTagPoseMsg2TagPose(awesomo_msgs::AprilTagPose msg);

}  // end of awesomo namespace
#endif
