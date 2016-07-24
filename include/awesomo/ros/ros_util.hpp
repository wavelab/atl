#include <atim/AtimPoseStamped.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>


int geoMessageQuatToEigenQuat(
    const geometry_msgs::Quaternion &q,
    Eigen::Quaterniond &eigen_quat
);

int tfQuatToEigenQuat(
    tf::Quaternion q,
    Eigen::Quaterniond &eigen_quat
);




