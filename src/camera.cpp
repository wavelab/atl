#include "awesomo/camera.hpp"


static double tic(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}


Camera::Camera(int camera_index, const std::string calibration_fp)
{
    this->tag_detector = new AprilTags::TagDetector(AprilTags::tagCodes16h5);
    this->camera_index = camera_index;
    this->loadCalibrationFile(calibration_fp);
}

static int checkMatrixYaml(YAML::Node matrix_yaml)
{
    const std::string targets[3] = { "rows", "cols", "data" };

    // pre-check
    if (matrix_yaml == NULL) {
        return -1;
    }

    for (int i = 0; i < 3; i++) {
        if (!matrix_yaml[targets[i]]) {
            return -1;
        }
    }

    return 0;
}

static cv::Mat loadMatrixFromYaml(YAML::Node matrix_yaml)
{
    int rows;
    int cols;
    int index;
    double value;

    // load matrix
    rows = matrix_yaml["rows"].as<int>();
    cols = matrix_yaml["cols"].as<int>();
    cv::Mat mat(rows, cols, CV_64F);

    index = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            value = matrix_yaml["data"][index].as<double>();
            mat.at<double>(i, j) = value;
            index++;
        }
    }

    return mat;
}

void Camera::loadCalibrationFile(const std::string calibration_fp)
{
    YAML::Node config = YAML::LoadFile(calibration_fp);

    // image width
    if (config["image_width"]) {
        this->image_width = config["image_width"].as<int>();
    }

    // image height
    if (config["image_height"]) {
        this->image_height = config["image_height"].as<int>();
    }

    // camera matrix
    if (config["camera_matrix"]) {
        this->camera_matrix = loadMatrixFromYaml(config["camera_matrix"]);
    }

    // distortion coefficients
    if (config["distortion_coefficients"]) {
        this->distortion_coefficients = loadMatrixFromYaml(config["distortion_coefficients"]);
    }

    // rectification matrix
    if (config["rectification_matrix"]) {
        this->rectification_matrix = loadMatrixFromYaml(config["rectification_matrix"]);
    }

    // projection matrix
    if (config["projection_matrix"]) {
        this->projection_matrix = loadMatrixFromYaml(config["projection_matrix"]);
    }
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

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    Eigen::Matrix3d F;
    Eigen::Matrix3d fixed_rot;

    // setup
    F << 1, 0, 0,
         0, -1, 0,
         0, 0, 1;

    if (detection.id == 0) {
        m_tag_size = 0.048;
    } else {
        m_tag_size = 0.343;
    }

    // recovering the relative pose of a tag:
    detection.getRelativeTranslationRotation(
        m_tag_size,
        this->camera_matrix.at<double>(0, 0),
        this->camera_matrix.at<double>(1, 1),
        this->camera_matrix.at<double>(0, 2),
        this->camera_matrix.at<double>(1, 2),
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
    std::cout << "TAGS:" << this->apriltags.size() << std::endl;
    for (int i = 0; i < this->apriltags.size(); i++) {
        this->printDetection(this->apriltags[i]);
    }

    return this->apriltags.size();
}

int main(int argc, char **argv)
{
    Camera cam(1, "/home/chutsu/ost.yml");
    cam.run();
}
