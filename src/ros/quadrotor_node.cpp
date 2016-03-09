#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "awesomo.quadrotor");
    ros::Rate loop_rate(30);
    ros::NodeHandle n;
    ros::Publisher publisher;
    std_msgs::String msg;
    std::stringstream ss;

    publisher = n.advertise<std_msgs::String>("awesomo/quadrotor", 1000);

    while (ros::ok()) {
        ss << "Hello World";
        msg.data = ss.str();

        publisher.publish(msg);
        ros::spinOnce();
    }

    return 0;
}
