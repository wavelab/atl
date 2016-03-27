#include "awesomo/camera.hpp"


static double tic(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}

int Camera::initWebcam(int image_width, int image_height)
{
    // setup
    this->capture = new cv::VideoCapture(this->camera_index);

    // open
    if (this->capture->isOpened() == 0) {
        ROS_INFO("Failed to open webcam!");
        return -1;

    } else {
        this->capture->set(CV_CAP_PROP_FRAME_WIDTH, image_width);
        this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, image_height);

    }
    ROS_INFO("Camera initialized!");

    return 0;
}

int Camera::initFirefly()
{
    FlyCapture2::Error error;

    // setup
    this->capture_firefly = new FlyCapture2::Camera();

    // connect
    error = this->capture_firefly->Connect(0);
    if (error != FlyCapture2::PGRERROR_OK) {
        ROS_INFO("Failed to connect to camera!");
        return -1;
    } else {
        ROS_INFO("Firefly camera connected!");
    }

    // start camera
    error = this->capture_firefly->StartCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
        ROS_INFO("Failed start camera!");
        return -1;
    } else {
        ROS_INFO("Firefly initialized!");
    }

    return 0;
}

int Camera::initCamera(std::string camera_mode)
{
    // load calibration file
    if (loadConfig(camera_mode) == -1) {
        ROS_INFO("Failed to initialize camera!");
        return -1;
    }

    // intialize camera
    if (this->camera_type == CAMERA_NORMAL) {
        this->initWebcam(this->config->image_width, this->config->image_height);
    } else if (this->camera_type == CAMERA_FIREFLY) {
        this->initFirefly();
    } else {
        ROS_INFO("Invalid Camera Type: %d!", this->camera_type);
        ROS_INFO("Failed to initialize camera!");
    }

    return 0;
}

Camera::Camera(int camera_index, int camera_type)
{
    this->camera_index = camera_index;
    this->camera_type = camera_type;
    this->tag_detector = new AprilTags::TagDetector(AprilTags::tagCodes16h5);
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

CameraConfig *Camera::loadConfig(std::string mode, const std::string calib_file)
{
    CameraConfig *camera_config = new CameraConfig();

    try {
        YAML::Node config = YAML::LoadFile(calib_file);

        // image width
        if (config["image_width"]) {
            camera_config->image_width = config["image_width"].as<int>();
        } else {
            ROS_ERROR("Failed to load image_width");
        }

        // image height
        if (config["image_height"]) {
            camera_config->image_height = config["image_height"].as<int>();
        } else {
            ROS_ERROR("Failed to load image_height");
        }

        // camera matrix
        if (config["camera_matrix"]) {
            camera_config->camera_matrix = loadMatrixFromYaml(
                config["camera_matrix"]
            );
        } else {
            ROS_ERROR("Failed to load camera_matrix");
        }

        // distortion coefficients
        if (config["distortion_coefficients"]) {
            camera_config->distortion_coefficients = loadMatrixFromYaml(
                config["distortion_coefficients"]
            );
        } else {
            ROS_ERROR("Failed to load distortion_coefficients");
        }

        // rectification matrix
        if (config["rectification_matrix"]) {
            camera_config->rectification_matrix = loadMatrixFromYaml(
                config["rectification_matrix"]
            );
        } else {
            ROS_ERROR("Failed to load rectification_matrix");
        }

        // projection matrix
        if (config["projection_matrix"]) {
            camera_config->projection_matrix = loadMatrixFromYaml(
                config["projection_matrix"]
            );
        } else {
            ROS_ERROR("Failed to load proejection_matrix");
        }

    } catch (YAML::BadFile &ex) {
        ROS_ERROR(
            "Failed to load calibration file: %s",
            calib_file.c_str()
        );
        throw;
    }

    // add to configs
    this->configs[mode] = camera_config;

    return camera_config;
}

int Camera::loadConfig(std::string camera_mode)
{
    if (this->configs.find(camera_mode) != this->configs.end()) {
        this->camera_mode = camera_mode;
        this->config = this->configs.find(camera_mode)->second;
        this->roi_rect = cv::Rect(
            0,
            0,
            config->image_width,
            config->image_height
        );
    } else {
        ROS_INFO("Config file for mode [%s] not found!", camera_mode.c_str());
        return -1;
    }

    return 0;
}

int Camera::getFrame(cv::Mat &image)
{
    unsigned int row_bytes;
    double data_size;
    double data_rows;
    FlyCapture2::Image raw_img;
    FlyCapture2::Image rgb_img;
    FlyCapture2::Error error;

    if (this->camera_type == CAMERA_NORMAL) {
        this->capture->read(image);

    } else if (this->camera_type == CAMERA_FIREFLY) {
        // get the image
        error = this->capture_firefly->RetrieveBuffer(&raw_img);
        if (error != FlyCapture2::PGRERROR_OK) {
            ROS_INFO("Video capture error!");
            return -1;
        }

        // convert to rgb
        raw_img.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgb_img);

        // convert to opencv mat
        data_size = rgb_img.GetReceivedDataSize();
        data_rows = rgb_img.GetRows();
        row_bytes = data_size / data_rows;
        cv::Mat(
            rgb_img.GetRows(),
            rgb_img.GetCols(),
            CV_8UC3,
            rgb_img.GetData(),
            row_bytes
        ).copyTo(image);

    } else {
        ROS_INFO("Invalid Camera Type: %d!", camera_type);
        return -2;
    }

    return 0;
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

AprilTagPose Camera::obtainAprilTagPose(AprilTags::TagDetection &detection)
{
    AprilTagPose pose;
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
    } else if (detection.id == 5) {
        m_tag_size = 0.343;
    }

    // recovering the relative pose of a tag:
    detection.getRelativeTranslationRotation(
        m_tag_size,
        this->config->camera_matrix.at<double>(0, 0),
        this->config->camera_matrix.at<double>(1, 1),
        this->config->camera_matrix.at<double>(0, 2),
        this->config->camera_matrix.at<double>(1, 2),
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
    AprilTagPose pose;

    pose = this->obtainAprilTagPose(detection);
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


cv::Rect enlargeROI(cv::Mat& frm, cv::Rect boundingBox, int padding)
{
    cv::Rect returnRect = cv::Rect(boundingBox.x - padding, boundingBox.y - padding,
                                   boundingBox.width + (padding * 2),
                                   boundingBox.height + (padding * 2)
                          );
    // check the size of the roi
    if (returnRect.x < 0) returnRect.x = 0;
    if (returnRect.y < 0) returnRect.y = 0;
    if (returnRect.x + returnRect.width >= frm.cols){
        returnRect.width = frm.cols-returnRect.x;
    }
    if (returnRect.y + returnRect.height >= frm.rows){
        returnRect.height = frm.rows - returnRect.y;
    }

    return returnRect;
}

void Camera::adjustCameraMode(
    std::vector<AprilTagPose> &pose_estimates,
    int &timeout
)
{
    AprilTagPose pose;

    // pre-check
    if (timeout > 5 && this->camera_mode == "160") {
        ROS_INFO("timeout!!");
        this->loadConfig("320");
        timeout = 0;
        return;
    }

    // adjust
    if (pose_estimates.size() == 0) {
        timeout++;
    } else {
        pose = pose_estimates[0];
        if (this->camera_mode == "320" && pose.translation[0] <= 1.5) {
            this->loadConfig("160");
        } else if (this->camera_mode == "160" && pose.translation[0] >= 1.8) {
            this->loadConfig("320");
        }
    }
}

std::vector<AprilTagPose> Camera::processImage(cv::Mat &image, int &timeout)
{
    cv::Point2f p1;
    cv::Point2f p2;
    cv::Mat image_undistort;
    cv::Mat image_gray;

    AprilTagPose pose;
    std::vector<AprilTagPose> pose_estimates;

    // apriltags detector (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    cv::resize(
        image_gray,
        image_gray,
        cv::Size(this->config->image_width, this->config->image_height)
    );

    // create a mask and draw the roi_rect
    cv::Mat mask(image_gray.rows, image_gray.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat masked(image_gray.rows, image_gray.cols, CV_8UC1, cv::Scalar(0));
    cv::rectangle(mask, this->roi_rect, 255, -1);
    image_gray.copyTo(masked, mask);

    // extract apriltags and estimate pose
    this->apriltags = this->tag_detector->extractTags(masked);
    for (int i = 0; i < this->apriltags.size(); i++) {
        if (this->apriltags[i].id == 5 || this->apriltags[i].id == 0) {
            // this->printDetection(this->apriltags[i]);
            pose = this->obtainAprilTagPose(this->apriltags[i]);
            pose_estimates.push_back(pose);

            // calc the roi rect
            p1 = cv::Point2f(this->apriltags[i].p[1].first, this->apriltags[i].p[1].second);
            p2 = cv::Point2f(this->apriltags[i].p[3].first, this->apriltags[i].p[3].second);
            float x = this->apriltags[i].cxy.first;
            float y = this->apriltags[i].cxy.second;
            float normdist = cv::norm(p2 - p1);
            this->roi_rect = cv::Rect(x-normdist/2, y-normdist/2, normdist, normdist);
            this->roi_rect = enlargeROI(image_gray, this->roi_rect, 5);

            // only need 1 tag
            timeout = 0;
            break;
        }
    }

    // enlarge the roi rect so to be the size of the image
    if (this->apriltags.size() == 0) {
        this->roi_rect = enlargeROI(image_gray, this->roi_rect, 1000);
    }

    // adjust camera mode
    this->adjustCameraMode(pose_estimates, timeout);

    // display result
    // cv::imshow("camera", masked);
    // cv::waitKey(1);

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

int Camera::outputAprilTagPose(const std::string output_fp, AprilTagPose &pose)
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
    int frame_index;
    int timeout;
    double last_tic;
    cv::Mat image;
    std::vector<AprilTagPose> pose_estimates;

    // setup
    timeout = 0;
    frame_index = 0;
    last_tic = tic();

    // read capture device
    while (true) {
        this->getFrame(image);
        pose_estimates = this->processImage(image, timeout);
        // this->printFPS(last_tic, frame_index);
        // cv::imshow("camera", image);
        // cv::waitKey(1);
    }

    return 0;
}

std::vector<AprilTagPose> Camera::step(int &timeout)
{
    cv::Mat image;

    this->getFrame(image);
    return this->processImage(image, timeout);
}
