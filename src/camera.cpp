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
    if (this->loadConfig(camera_mode) == -1) {
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
    ROS_INFO("Camera is running...");

    return 0;
}

Camera::Camera(int camera_index, int camera_type)
{
    this->camera_index = camera_index;
    this->camera_type = camera_type;
    this->tag_detector =  new TagDetector();
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

        // resize the image to reflect camera mode
        cv::resize(
            image,
            image,
            cv::Size(
                this->config->image_width,
                this->config->image_height
            )
        );

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

void Camera::adjustMode(std::vector<TagPose> &pose_estimates, int &timeout)
{
    TagPose pose;

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
            timeout = 0;
        } else if (this->camera_mode == "160" && pose.translation[0] >= 1.8) {
            this->loadConfig("320");
            timeout = 0;
        }
    }
}

int Camera::run(void)
{
    int frame_index;
    int timeout;
    double last_tic;
    cv::Mat image;
    std::vector<TagPose> pose_estimates;

    // setup
    timeout = 0;
    frame_index = 0;
    last_tic = tic();

    // read capture device
    while (true) {
        this->getFrame(image);
        pose_estimates = this->tag_detector->processImage(
            this->config->camera_matrix,
            image,
            timeout
        );
        this->adjustMode(pose_estimates, timeout);

        // this->printFPS(last_tic, frame_index);
        // cv::imshow("camera", image);
        // cv::waitKey(1);

    }

    return 0;
}

std::vector<TagPose> Camera::step(int &timeout)
{
    cv::Mat image;
    std::vector<TagPose> pose_estimates;


    this->getFrame(image);

    pose_estimates = this->tag_detector->processImage(
        this->config->camera_matrix,
        image,
        timeout
    );
    this->adjustMode(pose_estimates, timeout);

    return pose_estimates;
}

int Camera::photoMode(void)
{
    int frame_index;
    int timeout;
    double last_tic;
    cv::Mat image;
    std::vector<TagPose> pose_estimates;

    int key_input;
    cv::Mat image_cap;
    int image_number;

    // setup
    timeout = 0;
    frame_index = 0;
    last_tic = tic();

    image_number = 1;

    // read capture device
    while (true) {
        this->getFrame(image);
        pose_estimates = this->tag_detector->processImage(
            this->config->camera_matrix,
            image,
            timeout
        );
        this->adjustMode(pose_estimates, timeout);

        // cv::imshow("camera", image);
        // cv::waitKey(1);

        key_input = cvWaitKey(100);
        if((char) key_input == 49){
            std::cout << "Saving a new image" << std::endl;
            image.copyTo(image_cap);
            imshow("image capture", image_cap);
        }
    }

    return 0;
}
