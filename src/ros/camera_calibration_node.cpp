#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "awesomo/camera.hpp"
#include <cv_bridge/cv_bridge.h>
int main(int argc, char **argv)
{
    int seq;
    ros::init(argc, argv, "awesomo_camera");
    ros::NodeHandle n;
    ros::Rate rate(100);
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("awesomo/camera/image", 30);

    // setup
    Camera cam(
        0,
        CAMERA_FIREFLY,
        "/home/stan/Projects/awesomo/configs/ost.yml"
     //   "/home/chutsu/Dropbox/proj/awesomo/configs/ost.yml"
    );
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(30);
    cv::Mat image;
    cv::Mat result;
    while (ros::ok()) {
        // obtain image
        cam.getFrame(image);
        cv::resize(image, result, cv::Size(640/4, 480/4));
        msg = cv_bridge::CvImage(std_msgs::Header(),
                                    "rgb8", result).toImageMsg();
        // publish poses
        // publish and spin
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
