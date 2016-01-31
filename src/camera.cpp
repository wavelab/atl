#include "awesomo/camera.hpp"


static double tic(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}

Camera::Camera(void)
{
    this->camera_index = 1;
    this->image_width = 400;
    this->image_height = 400;
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

int Camera::run(void)
{
    int frame_index;
    double last_tic;
    cv::Mat image;
    cv::Mat image_gray;
    cv::VideoCapture capture(this->camera_index);

    // setup
    frame_index = 0;
    last_tic = tic();

    // open capture device
    if (capture.isOpened() == 0) {
        std::cout << "Failed to open webcam!";
        return -1;
    } else {
        capture.set(CV_CAP_PROP_FRAME_WIDTH, this->image_width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, this->image_height);
    }

    // read capture device
    while (true) {
        capture.read(image);

        this->processImage(image, image_gray);
        this->printFPS(last_tic, frame_index);

        if (cv::waitKey(30) >= 0) {
            break;
        }
    }

    return 0;
}

int Camera::processImage(cv::Mat &image, cv::Mat &image_gray)
{
    cv::Mat image_bw;
    int bw_threshold;

    // detect april tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    bw_threshold = 128;
    image_bw = image_gray > bw_threshold;
    // cv::adaptiveThreshold(
    //     image_gray,
    //     image_gray,
    //     255,
    //     CV_ADAPTIVE_THRESH_GAUSSIAN_C,
    //     CV_THRESH_BINARY,
    //     15,
    //     -5
    // );
    //
    cv::imshow("camera", image_bw);
    this->apriltags = this->tag_detector->extractTags(image_bw);

    // print out each apriltags
    std::cout << this->apriltags.size() << " tags detected:" << endl;

    return this->apriltags.size();
}

int main(int argc, char **argv)
{
    Camera cam;
    cam.run();
}
