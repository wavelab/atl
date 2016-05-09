#include "camera.hpp"


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
        printf("Failed to open webcam!\n");
        return -1;

    } else {
        this->capture->set(CV_CAP_PROP_FRAME_WIDTH, image_width);
        this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, image_height);

    }
    printf("Camera initialized!");

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
        printf("Failed to connect to camera!\n");
        return -1;
    } else {
        printf("Firefly camera connected!\n");
    }

    // start camera
    error = this->capture_firefly->StartCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
        printf("Failed start camera!\n");
        return -1;
    } else {
        printf("Firefly initialized!\n");
    }

    return 0;
}

int Camera::initCamera(std::string camera_mode)
{
    // load calibration file
    if (this->loadConfig(camera_mode) == -1) {
        printf("Failed to initialize camera!\n");
        return -1;
    }

    // intialize camera
    if (this->camera_type == CAMERA_NORMAL) {
        this->initWebcam(this->config->image_width, this->config->image_height);
    } else if (this->camera_type == CAMERA_FIREFLY) {
        this->initFirefly();
    } else {
        printf("Invalid Camera Type: %d!\n", this->camera_type);
        printf("Failed to initialize camera!\n");
    }
    printf("Camera is running...\n");

    return 0;
}

Camera::Camera(int camera_index, int camera_type)
{
    this->camera_index = camera_index;
    this->camera_type = camera_type;
    this->camera_snapshot = 0;
    this->tag_detector =  new TagDetector();

}

Camera::Camera(std::string camera_config_path)
{
    int nb_configs;
    std::string config_key;
    std::string config_path;
    YAML::Node camera_config;

    // load camera_config yaml file
    camera_config = YAML::LoadFile(camera_config_path + "/camera_config.yaml");
	nb_configs = camera_config["nb_configs"].as<int>();

    // create camera object
    this->camera_index = camera_config["camera_index"].as<int>();
    this->camera_imshow = camera_config["camera_imshow"].as<int>();
    this->camera_snapshot = camera_config["camera_snapshot"].as<int>();
    this->tag_detector =  new TagDetector(camera_config["apriltag_imshow"].as<int>());

	if (camera_config["camera_type"].as<std::string>() == "firefly") {
        this->camera_type = CAMERA_FIREFLY;
	} else {
        this->camera_type = CAMERA_NORMAL;
	}

    // load different calibration files
	for (int i = 0; i < nb_configs; i++) {
	    config_key = camera_config["config_keys"][i].as<std::string>();
	    config_path = camera_config_path + "/";
	    config_path += camera_config["config_files"][i].as<std::string>();

        this->loadConfig(config_key, config_path);
	}

    // start camera with first calibration file
    this->initCamera(camera_config["config_keys"][0].as<std::string>());
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
            printf("Failed to load image_width\n");
        }

        // image height
        if (config["image_height"]) {
            camera_config->image_height = config["image_height"].as<int>();
        } else {
            printf("Failed to load image_height\n");
        }

        // camera matrix
        if (config["camera_matrix"]) {
            camera_config->camera_matrix = loadMatrixFromYaml(
                config["camera_matrix"]
            );
        } else {
            printf("Failed to load camera_matrix\n");
        }

        // distortion coefficients
        if (config["distortion_coefficients"]) {
            camera_config->distortion_coefficients = loadMatrixFromYaml(
                config["distortion_coefficients"]
            );
        } else {
            printf("Failed to load distortion_coefficients\n");
        }

        // rectification matrix
        if (config["rectification_matrix"]) {
            camera_config->rectification_matrix = loadMatrixFromYaml(
                config["rectification_matrix"]
            );
        } else {
            printf("Failed to load rectification_matrix\n");
        }

        // projection matrix
        if (config["projection_matrix"]) {
            camera_config->projection_matrix = loadMatrixFromYaml(
                config["projection_matrix"]
            );
        } else {
            printf("Failed to load proejection_matrix\n");
        }

    } catch (YAML::BadFile &ex) {
        printf(
            "Failed to load calibration file: %s\n",
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
        printf("Loaded config file [%s]\n", camera_mode.c_str());

    } else {
        printf("Config file for mode [%s] not found!\n", camera_mode.c_str());
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
            printf("Video capture error!\n");
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
        printf("Invalid Camera Type: %d!\n", camera_type);
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
        printf("timeout!!\n");
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
    int timeout;
    cv::Mat image;
    cv::Mat image_capture;
    std::vector<TagPose> pose_estimates;

    // setup
    timeout = 0;

    // read capture device
    while (true) {
        this->step(timeout);
    }

    return 0;
}

std::vector<TagPose> Camera::step(int &timeout)
{
    cv::Mat image;
    std::vector<TagPose> pose_estimates;

    // get frame and apriltag pose estimates
    this->getFrame(image);
    pose_estimates = this->tag_detector->processImage(
        this->config->camera_matrix,
        image,
        timeout
    );
    this->adjustMode(pose_estimates, timeout);

    // imshow
    if (this->camera_imshow) {
        cv::imshow("camera", image);
        cv::waitKey(1);
    }

    // snapshot
    // if (this->camera_snapshot && (char) cv::waitKey(100) == 32) {
    //     std::cout << "Saving a new image" << std::endl;
    //     cv::imshow("image capture", image);
    //     cv::imwrite("image.jpg", image);
    // }

    return pose_estimates;
}
