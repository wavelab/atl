#include "awesomo/camera.hpp"


static double tic(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}


Camera::Camera(void)
{
    this->camera_index = 0;
    this->image_width = 400;
    this->image_height = 400;

    this->focal_length = 4.0;
    this->field_of_view = 60;
    // this->ccd_width = NULL;

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

float Camera::calculateFocalLength(void)
{
    if (this->ccd_width) {
        return this->image_width * this->focal_length / this->ccd_width;

    } else if (this->field_of_view) {
        return (this->image_width * 0.5) / tan(this->field_of_view * 0.5 * M_PI / 180);

    } else {
        return -1;

    }
}

double Camera::standardRad(double t)
{
    // normalize angle to be within the interval [-pi,pi].
    if (t >= 0.) {
        t = fmod(t + M_PI, TWOPI) - M_PI;
    } else {
        t = fmod(t - M_PI, -TWOPI) + M_PI;
    }

    return t;
}

void Camera::convertToEuler(
    const Eigen::Matrix3d &wRo,
    double &yaw,
    double &pitch,
    double &roll
)
{
    yaw = this->standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = this->standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = this->standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

void Camera::printDetection(AprilTags::TagDetection& detection)
{
    double yaw;
    double pitch;
    double roll;

    double m_tag_size;
    double m_fx;
    double m_fy;
    double m_px;
    double m_py;

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    Eigen::Matrix3d F;
    Eigen::Matrix3d fixed_rot;

    // setup
    F << 1, 0, 0,
         0, -1, 0,
         0, 0, 1;

    m_tag_size = 0.343;
    m_fx = this->calculateFocalLength();
    m_fy = this->calculateFocalLength();
    m_px = this->image_width / 2.0;
    m_py = this->image_height / 2.0;

    // recovering the relative pose of a tag:
    detection.getRelativeTranslationRotation(
        m_tag_size,
        m_fx,
        m_fy,
        m_px,
        m_py,
        translation,
        rotation
    );

    fixed_rot = F * rotation;
    this->convertToEuler(fixed_rot, yaw, pitch, roll);

    std::cout << "id: " << detection.id << " ";
    std::cout << "Hamming: " << detection.hammingDistance << ")";
    std::cout << "  distance=" << translation.norm() << "m ";
    std::cout << "x=" << translation(0) << ", ";
    std::cout << "y=" << translation(1) << ", ";
    std::cout << "z=" << translation(2);
    std::cout << "yaw=" << yaw << ", ";
    std::cout << "pitch=" << pitch << ", ";
    std::cout << "roll=" << roll << ", ";
    std::cout << endl;

    // also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.
}

int Camera::processImage(cv::Mat &image, cv::Mat &image_gray)
{
    // detect april tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    cv::imshow("camera", image_gray);
    this->apriltags = this->tag_detector->extractTags(image_gray);

    // print out each apriltags
    if (this->apriltags.size()) {
        this->printDetection(this->apriltags[0]);
    }

    return this->apriltags.size();
}

int main(int argc, char **argv)
{
    Camera cam;
    cam.run();
}
