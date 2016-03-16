#include "awesomo/camera.hpp"



static double tic(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}


Camera::Camera(int camera_index, int camera_type, const std::string calibration_fp)
{
    FlyCapture2::Error error;

    // initlize apriltag detector
    this->tag_detector = new AprilTags::TagDetector(AprilTags::tagCodes16h5);

    // intialize camera
    this->camera_index = camera_index;
    this->camera_type = camera_type;

    if (this->camera_type == CAMERA_NORMAL) {
        this->loadCalibrationFile(calibration_fp);
        this->capture = new cv::VideoCapture(camera_index);

        if (this->capture->isOpened() == 0) {
            ROS_INFO("Failed to open webcam!");
        } else {
            this->capture->set(CV_CAP_PROP_FRAME_WIDTH, this->image_width);
            this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, this->image_height);
        }
        ROS_INFO("Camera initialized!");

    } else if (this->camera_type == CAMERA_FIREFLY) {
        this->capture_firefly = new FlyCapture2::Camera();

        error = this->capture_firefly->Connect(0);
        if (error != FlyCapture2::PGRERROR_OK) {
            ROS_INFO("Failed to connect to camera!");
        } else {
            ROS_INFO("Firefly camera connected!");
        }

        error = this->capture_firefly->StartCapture();
        if (error != FlyCapture2::PGRERROR_OK) {
            ROS_INFO("Failed start camera!");
        } else {
            ROS_INFO("Firefly initialized!");
        }

    } else {
        ROS_INFO("Invalid Camera Type: %d!", camera_type);
    }
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
    try {
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
            this->camera_matrix = loadMatrixFromYaml(
                config["camera_matrix"]
            );
        }

        // distortion coefficients
        if (config["distortion_coefficients"]) {
            this->distortion_coefficients = loadMatrixFromYaml(
                config["distortion_coefficients"]
            );
        }

        // rectification matrix
        if (config["rectification_matrix"]) {
            this->rectification_matrix = loadMatrixFromYaml(
                config["rectification_matrix"]
            );
        }

        // projection matrix
        if (config["projection_matrix"]) {
            this->projection_matrix = loadMatrixFromYaml(
                config["projection_matrix"]
            );
        }
    } catch (YAML::BadFile &ex) {
        throw;
    }
}

void Camera::getFrame(cv::Mat &image)
{

    if (this->camera_type == CAMERA_NORMAL) {
        // this->capture->read(image);

    } else if (this->camera_type == CAMERA_FIREFLY) {
        unsigned int row_bytes;
        FlyCapture2::Image raw_img;
        FlyCapture2::Image rgb_img;
        FlyCapture2::Error error;

        // get the image
        error = this->capture_firefly->RetrieveBuffer(&raw_img);
        if (error != FlyCapture2::PGRERROR_OK) {
            ROS_INFO("Video capture error!");
        }

        // convert to rgb
        raw_img.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgb_img);

        // convert to opencv mat
        row_bytes = (double) rgb_img.GetReceivedDataSize() / (double) rgb_img.GetRows();
        cv::Mat(
            rgb_img.GetRows(),
            rgb_img.GetCols(),
            CV_8UC3,
            rgb_img.GetData(),
            row_bytes
        ).copyTo(image);

    } else {
        ROS_INFO("Invalid Camera Type: %d!", camera_type);

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

PoseEstimate Camera::obtainPoseEstimate(AprilTags::TagDetection& detection)
{
    PoseEstimate pose;
    double m_tag_size;

    Eigen::Matrix3d F;
    Eigen::Matrix3d rotation;
    Eigen::Matrix3d fixed_rot;

    // setup
    F << 1, 0, 0,
         0, -1, 0,
         0, 0, 1;

    // change tag size according to tag id
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
        pose.translation,
        rotation
    );
    fixed_rot = F * rotation;
    pose.distance = pose.translation.norm();
    this->convertToEuler(fixed_rot, pose.yaw, pose.pitch, pose.roll);

    return pose;
}

void Camera::printDetection(AprilTags::TagDetection& detection)
{
    PoseEstimate pose;

    pose = this->obtainPoseEstimate(detection);
    ROS_INFO("id: %d ", detection.id);
    ROS_INFO("Hamming: %d ", detection.hammingDistance);
    ROS_INFO("distance= %fm ", pose.translation.norm());
    ROS_INFO("x=%f ", pose.translation(0));
    ROS_INFO("y=%f ", pose.translation(1));
    ROS_INFO("z=%f ", pose.translation(2));
    ROS_INFO("yaw=%f ", pose.yaw);
    ROS_INFO("pitch=%f ", pose.pitch);
    ROS_INFO("roll=%f \n", pose.roll);

    // also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.
}

std::vector<PoseEstimate> Camera::processImage(cv::Mat &image, cv::Mat &image_gray)
{
    PoseEstimate pose;
    std::vector<PoseEstimate> pose_estimates;

    // detect april tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    this->apriltags = this->tag_detector->extractTags(image_gray);

    // print out each apriltags
    for (int i = 0; i < this->apriltags.size(); i++) {
        this->printDetection(this->apriltags[i]);
        pose = this->obtainPoseEstimate(this->apriltags[i]);
        pose_estimates.push_back(pose);
        this->apriltags[i].draw(image);
    }

    return pose_estimates;
}

bool Camera::isFileEmpty(const std::string file_path)
{
    std::ifstream f(file_path);

    if (f && f.peek() == std::ifstream::traits_type::eof()) {
        f.close();
        return true;
    } else if (!f) {
        return true;
    } else {
        return false;
    }
}

int Camera::outputPoseEstimate(const std::string output_fp, PoseEstimate &pose)
{
    bool empty_file;
    std::ofstream output_file;

    // setup
    empty_file = this->isFileEmpty(output_fp);
    output_file.open(output_fp, ios::out | ios::app);

    if (output_file.is_open()) {
        if (empty_file) {
            output_file << "distance" << ", ";
            output_file << "yaw" << ", ";
            output_file << "pitch" << ", ";
            output_file << "roll" << ", ";
            output_file << "x" << ", ";
            output_file << "y" << ", ";
            output_file << "z" << std::endl;
        }

        output_file << pose.translation.norm() << ", ";
        output_file << pose.yaw << ", ";
        output_file << pose.pitch << ", ";
        output_file << pose.roll << ", ";
        output_file << pose.translation(0) << ", ";
        output_file << pose.translation(1) << ", ";
        output_file << pose.translation(2) << std::endl;
        output_file.close();
    } else {
        return -1;
    }

    return 0;
}

int Camera::run(void)
{
    int c;
    int frame_index;
    double last_tic;
    cv::Mat image;
    cv::Mat image_gray;
    std::vector<PoseEstimate> pose_estimates;

    // setup
    frame_index = 0;
    last_tic = tic();

    // read capture device
    while (true) {
        this->getFrame(image);
        // pose_estimates = this->processImage(image, image_gray);

        this->printFPS(last_tic, frame_index);
        // cv::imshow("camera", image);
        // cv::waitKey(1);
    }

    return 0;
}

// std::vector<PoseEstimate> Camera::step(void)
// {
//     cv::Mat image;
//     cv::Mat image_gray;
//
//     this->getFrame(image);
//     return this->processImage(image, image_gray);
// }
//
// int Camera::runCalibration(void)
// {
//     int c;
//     int frame_index;
//     double last_tic;
//     cv::Mat image;
//     cv::Mat image_gray;
//     std::vector<PoseEstimate> pose_estimates;
//
//     // setup
//     frame_index = 0;
//     last_tic = tic();
//
//     while (true) {
//         c = cv::waitKey(100);
//         this->getFrame(image);
//
//         pose_estimates = this->processImage(image, image_gray);
//         this->printFPS(last_tic, frame_index);
//
//         if (c == 113) {
//             ROS_INFO("exiting...");
//             break;
//
//         } else if (c == 'c') {
//             if (pose_estimates.size()) {
//                 ROS_INFO("record pose estimate!");
//                 this->outputPoseEstimate("pose_estimate.dat", pose_estimates[0]);
//             } else {
//                 ROS_INFO("no apriltags detected!");
//             }
//         }
//     }
//
//     return 0;
// }
