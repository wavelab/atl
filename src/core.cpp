#include "awesomo/core.hpp"


void imu_orientation_cb(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("GOT: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "awesomo_imu_orientation");

    ros::Subscriber imu_orientation_sub;
    ros::NodeHandle imu_orientation;

    // setup
    imu_orientation_sub = imu_orientation.subscribe(
        "/mavros/imu/data/orientation",
        1000,
        imu_orientation_cb
    );
    ros::spin();

    return 0;
}
