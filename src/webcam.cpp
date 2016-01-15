#include "webcam.hpp"


void webcam_run(void)
{
    cv::VideoCapture capture(1);
    cv::Mat frame;

    /* open capture device */
    if (capture.isOpened() != 0) {
        std::cout << "Failed to open webcam!";
    }

    /* read capture device */
    while (true) {
        capture.read(frame);
        cv::imshow("camera", frame);

        if (cv::waitKey(30) >= 0) {
            return;
        }
    }
}
