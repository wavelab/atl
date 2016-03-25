#include "awesomo/camera.hpp"


static double tic(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}

int Camera::initializeNormalCamera()
{
    // setup
    this->capture = new cv::VideoCapture(this->camera_index);

    // open
    if (this->capture->isOpened() == 0) {
        ROS_INFO("Failed to open webcam!");
        return -1;

    } else {
        this->capture->set(CV_CAP_PROP_FRAME_WIDTH, this->image_width);
        this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, this->image_height);

    }
    ROS_INFO("Camera initialized!");

    return 0;
}

int Camera::initializeFireflyCamera()
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

Camera::Camera(int camera_index, int camera_type, const std::string calibration_fp)
{
    // initlize apriltag detector
    this->tag_detector = new AprilTags::TagDetector(AprilTags::tagCodes16h5);

    // intialize camera
    this->roi_rect = cv::Rect(0, 0, 640/2, 480/2);
    this->camera_index = camera_index;
    this->camera_type = camera_type;
    this->loadCalibrationFile(calibration_fp);

    if (this->camera_type == CAMERA_NORMAL) {
        this->initializeNormalCamera();

    } else if (this->camera_type == CAMERA_FIREFLY) {
        this->initializeFireflyCamera();

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
        } else {
            ROS_ERROR("Failed to load image_width");
        }

        // image height
        if (config["image_height"]) {
            this->image_height = config["image_height"].as<int>();
        } else {
            ROS_ERROR("Failed to load image_height");
        }

        // camera matrix
        if (config["camera_matrix"]) {
            this->camera_matrix = loadMatrixFromYaml(
                config["camera_matrix"]
            );
        } else {
            ROS_ERROR("Failed to load camera_matrix");
        }

        // distortion coefficients
        if (config["distortion_coefficients"]) {
            this->distortion_coefficients = loadMatrixFromYaml(
                config["distortion_coefficients"]
            );
        } else {
            ROS_ERROR("Failed to load distortion_coefficients");
        }

        // rectification matrix
        if (config["rectification_matrix"]) {
            this->rectification_matrix = loadMatrixFromYaml(
                config["rectification_matrix"]
            );
        } else {
            ROS_ERROR("Failed to load rectification_matrix");
        }

        // projection matrix
        if (config["projection_matrix"]) {
            this->projection_matrix = loadMatrixFromYaml(
                config["projection_matrix"]
            );
        } else {
            ROS_ERROR("Failed to load proejection_matrix");
        }

    } catch (YAML::BadFile &ex) {
        ROS_ERROR(
            "Failed to load calibration file: %s",
            calibration_fp.c_str()
        );
        throw;
    }
}

int Camera::getFrame(cv::Mat &image)
{
    unsigned int row_bytes;
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

AprilTagPose Camera::obtainAprilTagPose(AprilTags::TagDetection& detection)
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

std::vector<AprilTagPose> Camera::processImage(cv::Mat &image, cv::Mat &image_gray)
{
    AprilTagPose pose;
    std::vector<AprilTagPose> pose_estimates;
    cv::Mat image_undistort;
    cv::Mat empty;
    cv::Point2f p1;
    cv::Point2f p2;

    // detect april tags (requires a gray scale image)
    // cv::undistort(
    //     image,
    //     image_undistort,
    //     this->camera_matrix,
    //     this->distortion_coefficients
    // );
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    cv::resize(image_gray, image_gray, cv::Size(640/4, 480/4));

    //create a draw a mask
    cv::Mat mask(image_gray.rows, image_gray.cols, CV_8UC1, cv::Scalar(0));
    cv::rectangle(mask, this->roi_rect, 255, -1);
    cv::Mat result(image_gray.rows, image_gray.cols, CV_8UC1, cv::Scalar(0));
    image_gray.copyTo(result, mask);

    // cv::Mat hack = image_gray(this->roi_rect);

    imshow("test", result);
    cv::waitKey(1);
    std::pair<float, float> top_left_roi;
    float x1 = this->roi_rect.x;
    float y1 = this->roi_rect.y;
    this->apriltags = this->tag_detector->extractTags(result);


    // // print out each apriltags
    // for (int i = 0; i < this->apriltags.size(); i++) {
    if (this->apriltags.size()) {
        int i = 0;

        // this->apriltags[i].cxy.first += x1;
        // this->apriltags[i].cxy.second += y1;
        pose = this->obtainAprilTagPose(this->apriltags[i]);
        pose_estimates.push_back(pose);

        // this->apriltags[i].draw(image_gray);
        p1 = cv::Point2f(this->apriltags[i].p[1].first, this->apriltags[i].p[1].second);
        p2 = cv::Point2f(this->apriltags[i].p[3].first, this->apriltags[i].p[3].second);
        float x = this->apriltags[i].cxy.first;
        float y = this->apriltags[i].cxy.second;

        float normdist = cv::norm(p2 - p1);
        this->roi_rect = cv::Rect(x-normdist/2, y-normdist/2, normdist, normdist);

        this->roi_rect = enlargeROI(image_gray, this->roi_rect, 5);
        this->printDetection(this->apriltags[i]);
    }

    if (this->apriltags.size() == 0) {
        std::cout << "nothing detected" << std::endl;
        this->roi_rect = cv::Rect(0, 0, 639, 479);
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
    double last_tic;
    cv::Mat image;
    cv::Mat image_gray;
    std::vector<AprilTagPose> pose_estimates;

    // setup
    frame_index = 0;
    last_tic = tic();

    // read capture device
    while (true) {
        this->getFrame(image);
        pose_estimates = this->processImage(image, image_gray);
        this->printFPS(last_tic, frame_index);
        // cv::imshow("camera", image);
        // cv::waitKey(1);
    }

    return 0;
}

std::vector<AprilTagPose> Camera::step(void)
{
    cv::Mat image;
    cv::Mat image_gray;

    this->getFrame(image);
    return this->processImage(image, image_gray);
}

int Camera::runCalibration(void)
{
    int c;
    int frame_index;
    double last_tic;
    cv::Mat image;
    cv::Mat image_gray;
    std::vector<AprilTagPose> pose_estimates;

    // setup
    frame_index = 0;
    last_tic = tic();

    while (true) {
        c = cv::waitKey(1);
        this->getFrame(image);

        pose_estimates = this->processImage(image, image_gray);
        this->printFPS(last_tic, frame_index);

        if (c == 113) {
            ROS_INFO("exiting...");
            break;

        } else if (c == 'c') {
            if (pose_estimates.size()) {
                ROS_INFO("record pose estimate!");
                this->outputAprilTagPose("pose_estimate.dat", pose_estimates[0]);
            } else {
                ROS_INFO("no apriltags detected!");
            }
        }
    }

    return 0;
}
