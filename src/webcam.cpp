#include "webcam.h"


static double tic()
{
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

Camera::Camera()
{
    this->tag_detector = new AprilTags::TagDetector(AprilTags::tagCodes16h5);
}

void Camera::printFPS(double &last_tic, int &frame)
{
    frame++;
    if (frame % 10 == 0) {
        double t = tic();
        cout << "\t" << 10.0 / (t - last_tic) << " fps" << endl;
        last_tic = t;
    }
}

void Camera::run(void)
{
    int frame_index;
    double last_tic;
    cv::Mat image;
    cv::Mat image_gray;
    cv::VideoCapture capture(1);

    // setup
    frame_index = 0;
    last_tic = tic();

    // open capture device
    if (capture.isOpened() != 0) {
        std::cout << "Failed to open webcam!";
    }

    // read capture device
    while (true) {
        capture.read(image);

        // cv::adaptiveThreshold(
        //     image,
        //     image,
        //     255,
        //     CV_ADAPTIVE_THRESH_GAUSSIAN_C,
        //     CV_THRESH_BINARY,
        //     15,
        //     -5
        // );
        cv::imshow("camera", image);

        this->processImage(image, image_gray);
        this->printFPS(last_tic, frame_index);

        if (cv::waitKey(30) >= 0) {
            return;
        }
    }
}

int Camera::processImage(cv::Mat &image, cv::Mat &image_gray)
{
    vector<AprilTags::TagDetection> detection;

    // detect april tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    detection = this->tag_detector->extractTags(image_gray);

    // print out each detection
    std::cout << detection.size() << " tags detected:" << endl;

    return detection.size();
}
