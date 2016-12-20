#include "awesomo_core/vision/camera.hpp"


namespace awesomo {

static double time_now(void) {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}

static int checkMatrixYaml(YAML::Node matrix_yaml) {
  const std::string targets[3] = {"rows", "cols", "data"};

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

static cv::Mat loadMatrixFromYaml(YAML::Node matrix_yaml) {
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

static Eigen::MatrixXd loadEigenMatrixFromYaml(YAML::Node matrix_yaml) {
  int rows;
  int cols;
  int index;
  double value;

  // load matrix
  rows = matrix_yaml["rows"].as<int>();
  cols = matrix_yaml["cols"].as<int>();
  Eigen::MatrixXd mat(rows, cols);

  index = 0;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      value = matrix_yaml["data"][index].as<double>();
      mat(i, j) = value;
      index++;
    }
  }
  return mat;
}

Camera::Camera(int camera_index, int camera_type) {
  this->camera_index = camera_index;
  this->camera_type = camera_type;
  this->camera_snapshot = 0;
  this->camera_exposure_value = 0;
  this->tag_detector = new Detector();

  this->gimbal = NULL;
  this->tag_estimator_initialized = false;
}

Camera::Camera(std::string camera_config_path) {
  int retval;
  float dist;
  int nb_configs;
  int nb_apriltags;
  int apriltag_id;
  float apriltag_size;
  std::string config_key;
  std::string config_path;
  std::string config_main_path;
  YAML::Node camera_config;

  // load camera_config yaml file
  config_path = camera_config_path + "/camera_config.yaml";
  camera_config = YAML::LoadFile(config_path);
  nb_configs = camera_config["nb_configs"].as<int>();

  // camera type
  if (camera_config["camera_type"]) {
    if (camera_config["camera_type"].as<std::string>() == "firefly") {
      this->camera_type = CAMERA_FIREFLY;
    } else if (camera_config["camera_type"].as<std::string>() == "ximea") {
      this->camera_type = CAMERA_XIMEA;

    } else {
      this->camera_type = CAMERA_NORMAL;
    }
  } else {
    printf("camera_type missing in %s\n", config_path.c_str());
  }

  // camera index
  if (camera_config["camera_index"]) {
    this->camera_index = camera_config["camera_index"].as<int>();
  } else {
    printf("camera_index missing in %s\n", config_path.c_str());
  }

  // camera imshow
  if (camera_config["camera_imshow"]) {
    this->camera_imshow = camera_config["camera_imshow"].as<int>();
  } else {
    printf("camera_imshow missing in %s\n", config_path.c_str());
  }

  // camera snapshot
  if (camera_config["camera_snapshot"]) {
    this->camera_snapshot = camera_config["camera_snapshot"].as<int>();
  } else {
    printf("camera_snapshot missing in %s\n", config_path.c_str());
  }

  // camera exposure value
  if (camera_config["camera_exposure_value"]) {
    this->camera_exposure_value =
      camera_config["camera_exposure_value"].as<float>();
    // std::cout << this->camera_exposure_value << std::endl;
  } else {
    printf("camera_exposure_value missing in %s\n", config_path.c_str());
  }

  // camera gain value
  if (camera_config["camera_gain_value"]) {
    this->camera_gain_value = camera_config["camera_gain_value"].as<float>();
  } else {
    printf("camera_gain_value missing in %s\n", config_path.c_str());
  }

  // tag detector
  if (camera_config["apriltag_imshow"]) {
    this->tag_detector =
      new Detector(camera_config["apriltag_imshow"].as<int>());
  } else {
    printf("apriltag_imshow missing in %s\n", config_path.c_str());
  }

  // load tag parameters
  nb_apriltags = camera_config["nb_apriltags"].as<int>();
  for (int i = 0; i < nb_apriltags; i++) {
    apriltag_id = camera_config["apriltag_ids"][i].as<int>();
    apriltag_size = camera_config["apriltag_sizes"][i].as<float>();
    this->tag_detector->tag_sizes[apriltag_id] = apriltag_size;
  }

  // load different calibration files
  for (int i = 0; i < nb_configs; i++) {
    config_key = camera_config["config_keys"][i].as<std::string>();
    config_path = camera_config_path + "/";
    config_path += camera_config["config_files"][i].as<std::string>();

    this->loadConfig(config_key, config_path);
  }

  // load config distances
  for (int i = 0; i < (nb_configs - 1); i++) {
    dist = camera_config["config_dists"][i].as<float>();
    this->config_dists.push_back(dist);
  }

  // print config
  this->printConfig();

  // start camera with first calibration file
  retval =
    this->initCamera(camera_config["config_keys"][0].as<std::string>());
  if (retval == -1) {
    exit(-1);
  } else {
    printf("Camera is running...\n");
  }

  // tag estimator
  this->tag_estimator_initialized = false;
}

int Camera::initWebcam(int image_width, int image_height) {
  // setup
  this->capture = new cv::VideoCapture(this->camera_index);

  // open
  if (this->capture->isOpened() == 0) {
    printf("ERROR! Failed to open webcam!\n");
    return -1;

  } else {
    this->capture->set(CV_CAP_PROP_FRAME_WIDTH, image_width);
    this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, image_height);
    printf("Camera initialized!\n");
    return 0;
  }
}

int Camera::initFirefly() {
  FlyCapture2::Error error;
  FlyCapture2::Property property;
  FlyCapture2::Property gain_property;

  // setup
  this->capture_firefly = new FlyCapture2::Camera();

  // connect
  error = this->capture_firefly->Connect(0);
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed to connect to camera!\n");
    return -1;
  } else {
    printf("Firefly camera connected!\n");
  }

  // set video mode format and frame rate
  error = this->capture_firefly->SetVideoModeAndFrameRate(
    FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_60);
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed to set camera video mode and frame rate!\n");
    return -1;
  } else {
    printf("Firefly camera video mode and frame rate configured!\n");
  }

  // configure exposure
  property.type = FlyCapture2::AUTO_EXPOSURE;
  property.onOff = true;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = this->camera_exposure_value;  // exposure value

  error = this->capture_firefly->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed to configure camera exposure!\n");
    return -1;
  } else {
    printf("Firefly camera exposure configured!\n");
  }

  // configure gain
  property.type = FlyCapture2::GAIN;
  property.onOff = true;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = this->camera_gain_value;  // set the gain

  error = this->capture_firefly->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed to configure camera gain!\n");
    return -1;
  } else {
    printf("Firefly camera gain configured!\n");
  }

  // start camera
  error = this->capture_firefly->StartCapture();
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed start camera!\n");
    return -1;
  } else {
    printf("Firefly initialized!\n");
  }

  return 0;
}

int Camera::initXimea() {
  int time_us;
  int ds_type;
  int ds_rate;
  float gain_db;
  int img_format;

  XI_RETURN retval;

  // setup
  time_us = this->camera_exposure_value;
  ds_type = 0;
  ds_rate = 2;
  gain_db = this->camera_gain_value;
  img_format = XI_RGB24;

  this->ximea = NULL;
  retval = XI_OK;

  printf("Starting Ximea Camera\n");

  // open camera device
  retval = xiOpenDevice(0, &this->ximea);
  XIMEA_CHECK(retval, "xiOpenDevice");

  // exposure time
  retval = xiSetParamInt(this->ximea, XI_PRM_EXPOSURE, time_us);
  XIMEA_CHECK(retval, "xiSetParam (exposure time set)");

  // downsampling type
  retval = xiSetParamInt(this->ximea, XI_PRM_DOWNSAMPLING_TYPE, ds_type);
  XIMEA_CHECK(retval, "xiSetParam (downsampling type)");

  // downsampling
  retval = xiSetParamInt(this->ximea, XI_PRM_DOWNSAMPLING, ds_rate);
  XIMEA_CHECK(retval, "xiSetParam (downsampling rate)");

  // exposure gain
  retval = xiSetParamFloat(this->ximea, XI_PRM_GAIN, gain_db);
  XIMEA_CHECK(retval, "xiSetParam (gain)");

  // image format
  retval = xiSetParamInt(this->ximea, XI_PRM_IMAGE_DATA_FORMAT, img_format);
  XIMEA_CHECK(retval, "xiSetParam (image format)");

  // buffer policy
  // retval = xiSetParamInt(this->ximea, XI_PRM_BUFFER_POLICY, XI_BP_SAFE);
  // XIMEA_CHECK(retval, "xiSetParam (buffer policy)");

  // start acquisition
  retval = xiStartAcquisition(this->ximea);
  XIMEA_CHECK(retval, "xiStartAcquisition");

  // return
  printf("Ximea Camera Started Successfully\n");
  return 0;

ximea_error:
  if (this->ximea) {
    xiCloseDevice(this->ximea);
  }
  return -1;
}

int Camera::initCamera(std::string camera_mode) {
  int retval;

  // load calibration file
  if (this->loadConfig(camera_mode) == -1) {
    printf("ERROR! Failed to initialize camera!\n");
    return -1;
  }

  // intialize camera
  if (this->camera_type == CAMERA_NORMAL) {
    retval =
      this->initWebcam(this->config->image_width, this->config->image_height);
  } else if (this->camera_type == CAMERA_FIREFLY) {
    retval = this->initFirefly();
  } else if (this->camera_type == CAMERA_XIMEA) {
    retval = this->initXimea();
  } else {
    printf("ERROR! Invalid Camera Type: %d!\n", this->camera_type);
    printf("ERROR! Failed to initialize camera!\n");
  }

  return retval;
}

int Camera::initGimbal(std::string config_path) {
  this->gimbal = new Gimbal(config_path);
  return 0;
}

CameraConfig *Camera::loadConfig(std::string mode,
                                 const std::string calib_file) {
  CameraConfig *camera_config = new CameraConfig();

  // record config ordering
  this->config_keys.push_back(mode);
  this->config_values.push_back(calib_file);

  // load config
  try {
    YAML::Node config = YAML::LoadFile(calib_file);

    // image width
    if (config["image_width"]) {
      camera_config->image_width = config["image_width"].as<int>();
    } else {
      printf("ERROR! Failed to load image_width\n");
    }

    // image height
    if (config["image_height"]) {
      camera_config->image_height = config["image_height"].as<int>();
    } else {
      printf("ERROR! Failed to load image_height\n");
    }

    // camera matrix
    if (config["camera_matrix"]) {
      camera_config->camera_matrix =
        loadMatrixFromYaml(config["camera_matrix"]);
    } else {
      printf("ERROR! Failed to load camera_matrix\n");
    }

    // distortion coefficients
    if (config["distortion_coefficients"]) {
      camera_config->distortion_coefficients =
        loadMatrixFromYaml(config["distortion_coefficients"]);
    } else {
      printf("ERROR! Failed to load distortion_coefficients\n");
    }

    // rectification matrix
    if (config["rectification_matrix"]) {
      camera_config->rectification_matrix =
        loadMatrixFromYaml(config["rectification_matrix"]);
    } else {
      printf("ERROR! Failed to load rectification_matrix\n");
    }

    // projection matrix
    if (config["projection_matrix"]) {
      camera_config->projection_matrix =
        loadMatrixFromYaml(config["projection_matrix"]);
      camera_config->projection_matrix_eigen =
        loadEigenMatrixFromYaml(config["projection_matrix"]);

    } else {
      printf("ERROR! Failed to load projection_matrix\n");
    }

  } catch (YAML::BadFile &ex) {
    printf("ERROR! Failed to load calibration file: %s\n",
           calib_file.c_str());
    throw;
  }

  // add to configs
  this->configs[mode] = camera_config;

  return camera_config;
}

int Camera::loadConfig(std::string camera_mode) {
  // pre-check
  if (this->camera_mode == camera_mode) {
    return 0;
  }

  // load config
  if (this->configs.find(camera_mode) != this->configs.end()) {
    this->camera_mode = camera_mode;
    this->config = this->configs.find(camera_mode)->second;
    printf("Loaded config file [%s]\n", camera_mode.c_str());

  } else {
    printf("Config file for mode [%s] not found!\n", camera_mode.c_str());
    return -1;
  }

  return 0;
}

int Camera::setLambdas(float lambda_1, float lambda_2, float lambda_3) {
  // alpha is calculated from:
  // Maddern et al, 2013, Illumination invarent imaging
  this->lambda_1 = lambda_1;
  this->lambda_2 = lambda_2;
  this->lambda_3 = lambda_3;
  this->alpha = (lambda_1 * lambda_3 - lambda_1 * lambda_2) /
                (lambda_2 * lambda_3 - lambda_1 * lambda_2);

  return 0;
}

void Camera::adjustMode(std::vector<TagPose> &pose_estimates, int &timeout) {
  TagPose pose;
  std::string config_key;
  int config_index;
  int config_length;
  float config_dist;
  float tag_dist;

  // pre-check
  if (this->config_keys.size() == 1) {
    return;
  }

  // load default resolution if no apriltag detected
  if (timeout > 5 && this->camera_mode != this->config_keys.at(0)) {
    printf("timeout!!\n");
    this->loadConfig(this->config_keys.at(0));
    timeout = 0;
    return;
  } else if (pose_estimates.size() == 0) {
    timeout++;
    return;
  }

  // adjust
  pose = pose_estimates[0];
  tag_dist = pose.translation[0];
  config_length = this->config_keys.size();

  // find config mode index
  for (config_index = 0; config_index < config_length; config_index++) {
    config_key = this->config_keys.at(config_index);
    if (this->camera_mode == config_key) {
      break;
    }
  }

  // load appropriate camera configuration
  // when camera config is currently between largest and smallest config
  if (config_index > 0 && config_index < this->config_dists.size()) {
    // load larger camera config
    if (tag_dist >= this->config_dists.at(config_index - 1)) {
      this->loadConfig(this->config_keys.at(config_index - 1));

      // load smaller camera config
    } else if (tag_dist <= this->config_dists.at(config_index)) {
      this->loadConfig(this->config_keys.at(config_index + 1));
    }

    // when camera config is already at largest
  } else if (config_index == 0) {
    // load smaller camera config
    if (tag_dist <= this->config_dists.at(config_index)) {
      this->loadConfig(this->config_keys.at(config_index + 1));
    }

    // when camera config is already at smallest
  } else if (config_index == this->config_dists.size()) {
    // load larger camera config
    if (tag_dist >= this->config_dists.at(config_index - 1)) {
      this->loadConfig(this->config_keys.at(config_index - 1));
    }
  }
  timeout = 0;
}

void Camera::printConfig(void) {
  std::cout << "camera_index: " << this->camera_index << std::endl;
  std::cout << "camera_type: " << this->camera_type << std::endl;
  std::cout << "camera_imshow: " << this->camera_imshow << std::endl;
  std::cout << "camera_snapshot: " << this->camera_snapshot << std::endl;
  std::cout << "camera_exposure_value: " << this->camera_exposure_value
            << std::endl;
  std::cout << "camera_gain_value: " << this->camera_gain_value << std::endl;
  std::cout << "camera_mode: " << this->camera_mode << std::endl;
  std::cout << "lambda_1: " << this->lambda_1 << std::endl;
  std::cout << "lambda_2: " << this->lambda_2 << std::endl;
  std::cout << "lambda_3: " << this->lambda_3 << std::endl;
  std::cout << "alpha: " << this->alpha << std::endl;
}

void Camera::printFPS(double &last_tic, int &frame) {
  frame++;
  if (frame % 10 == 0) {
    double t = time_now();
    std::cout << "\t" << 10.0 / (t - last_tic) << " fps" << std::endl;
    last_tic = t;
  }
}

int Camera::getFrameWebcam(cv::Mat &image) {
  this->capture->read(image);
  return 0;
}

int Camera::getFramePointGrey(cv::Mat &image) {
  double data_size;
  double data_rows;
  unsigned int row_bytes;

  FlyCapture2::Image raw_img;
  FlyCapture2::Image rgb_img;
  FlyCapture2::Error error;

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
  cv::Mat(rgb_img.GetRows(),
          rgb_img.GetCols(),
          CV_8UC3,
          rgb_img.GetData(),
          row_bytes)
    .copyTo(image);

  // resize the image to reflect camera mode
  cv::resize(image,
             image,
             cv::Size(this->config->image_width, this->config->image_height));

  return 0;
}

int Camera::getFrameXimea(cv::Mat &image) {
  int retval;
  XI_IMG ximea_img;

  memset(&ximea_img, 0, sizeof(ximea_img));
  ximea_img.size = sizeof(XI_IMG);

  // get the image
  retval = xiGetImage(this->ximea, 1000, &ximea_img);
  if (retval != XI_OK) {
    printf("Error after xiGetImage (%d)\n", retval);
    return -1;

  } else {
    // when ximea frame is mono
    // cv::Mat(frame.height, frame.width, CV_8U, frame.bp);

    // when ximea frame is rgb color (XI_RGB24 ONLY)
    cv::Mat(ximea_img.height, ximea_img.width, CV_8UC3, ximea_img.bp)
      .copyTo(image);

    // resize the image to reflect camera mode
    cv::resize(
      image,
      image,
      cv::Size(this->config->image_width, this->config->image_height));
    return 0;
  }
}

int Camera::getFrame(cv::Mat &image) {
  switch (this->camera_type) {
    case CAMERA_NORMAL:
      return this->getFrameWebcam(image);
    case CAMERA_FIREFLY:
      return this->getFramePointGrey(image);
    case CAMERA_XIMEA:
      return this->getFrameXimea(image);
    default:
      printf("Invalid Camera Type: %d!\n", camera_type);
      return -2;
  }
}

int Camera::run(void) {
  int timeout;
  int frame_index;
  double last_tic;
  std::vector<TagPose> pose_estimates;

  // setup
  timeout = 0;
  frame_index = 0;
  last_tic = time_now();

  // read capture device
  while (true) {
    pose_estimates = this->step(timeout);
    this->printFPS(last_tic, frame_index);
  }

  return 0;
}


static void show_image(cv::Mat &image, int tags_detected) {
  int border;
  cv::Scalar red;
  cv::Scalar green;

  // setup
  border = 5;
  red = cv::Scalar(0, 0, 255);
  green = cv::Scalar(0, 255, 0);

  // create detection border
  if (tags_detected) {
    cv::copyMakeBorder(image,
                       image,
                       border,
                       border,
                       border,
                       border,
                       cv::BORDER_CONSTANT,
                       green);

  } else {
    cv::copyMakeBorder(
      image, image, border, border, border, border, cv::BORDER_CONSTANT, red);
  }

  // show image
  cv::imshow("camera", image);
  cv::waitKey(1);
}

static void snapshot(cv::Mat &image, std::vector<TagPose> &pose_estimates) {
  TagPose tag;

  std::cout << "Saving a new image" << std::endl;
  cv::imshow("image capture", image);
  cv::imwrite("/tmp/image.png", image);

  if (pose_estimates.size()) {
    tag = pose_estimates[0];
    std::cout << "x: " << tag.translation[0] << std::endl;
    std::cout << "y: " << tag.translation[1] << std::endl;
    std::cout << "z: " << tag.translation[2] << std::endl;
  }
}

static void tag_estimator_init(struct kf *estimator, Eigen::Vector3d tag) {
  Eigen::VectorXd mu(9);

  mu << tag(0), tag(1), tag(2),  // x, y, z
    0.0, 0.0, 0.0,               // velocity x, y, z
    0.0, 0.0, 0.0;               // acceleration x, y, z

  apriltag_kf_setup(estimator, mu);
}

void Camera::trackTarget(std::vector<TagPose> pose_estimates) {
  double dt;
  Eigen::Vector3d tag;
  Eigen::Vector3d tag_trans;

  if (pose_estimates.size()) {
    tag << pose_estimates[0].translation(0), pose_estimates[0].translation(1),
      pose_estimates[0].translation(2);
    this->gimbal->transformTargetPosition(tag, tag_trans);

    if (this->tag_estimator_initialized == false) {
      tag_estimator_init(&this->tag_estimator, tag_trans);
      this->tag_estimator_initialized = true;
      tic(&this->tag_estimator_last_updated);

    } else {
      dt = toc(&this->tag_estimator_last_updated);
      apriltag_kf_estimate(&this->tag_estimator, tag_trans, dt, true);
      tic(&this->tag_estimator_last_updated);
    }

    tag(0) = this->tag_estimator.mu(0);
    tag(1) = this->tag_estimator.mu(1);
    tag(2) = this->tag_estimator.mu(2);
    this->gimbal->trackTarget(tag);
    tic(&this->tag_last_seen);

  } else if (this->tag_estimator_initialized) {
    if (toc(&this->tag_last_seen) > 2.0) {
      this->gimbal->setGimbalAngles(0.0, 0.0, 0.0);

    } else {
      dt = toc(&this->tag_estimator_last_updated);
      apriltag_kf_estimate(&this->tag_estimator, tag_trans, dt, false);
      tic(&this->tag_estimator_last_updated);

      tag(0) = this->tag_estimator.mu(0);
      tag(1) = this->tag_estimator.mu(1);
      tag(2) = this->tag_estimator.mu(2);
      this->gimbal->trackTarget(tag);
    }
  }
}

std::vector<TagPose> Camera::step(int &timeout) {
  cv::Mat image;
  std::vector<TagPose> pose_estimates;

  // get frame
  if (this->getFrame(image) != 0) {
    return pose_estimates;
  }

  // get apriltag pose estimates
  cv::flip(image, image, -1);
  pose_estimates = this->tag_detector->processImage(
    this->config->camera_matrix, image, timeout);
  this->adjustMode(pose_estimates, timeout);

  // track target
  this->trackTarget(pose_estimates);

  // imshow
  if (this->camera_imshow && image.empty() == false) {
    show_image(image, pose_estimates.size());
  }

  // snapshot
  if (this->camera_snapshot && (char) cv::waitKey(1000) == 's') {
    snapshot(image, pose_estimates);
  }

  return pose_estimates;
}

std::vector<TagPose> Camera::step(int &timeout, float dt) {
  cv::Mat image;
  Eigen::Vector3d tag;
  Eigen::Vector3d tag_trans;
  TagPose tag_pose;
  std::vector<TagPose> pose_estimates;

  // get frame
  if (this->getFrame(image) != 0) {
    return pose_estimates;
  }

  // get apriltag pose estimates
  // cv::flip(image, image, -1);
  pose_estimates = this->tag_detector->processImage(
    this->config->camera_matrix, image, timeout, dt, this->alpha);
  this->adjustMode(pose_estimates, timeout);

  // track target
  this->trackTarget(pose_estimates);
  if (pose_estimates.size()) {
    tag_pose = pose_estimates[0];
    tag << tag_pose.translation(0), tag_pose.translation(1),
      tag_pose.translation(2);
    this->gimbal->transformTargetPosition(tag, tag_trans);

    pose_estimates.clear();
    tag_pose.translation = tag_trans;
    pose_estimates.push_back(tag_pose);
  }

  // imshow
  if (this->camera_imshow) {
    show_image(image, pose_estimates.size());
  }

  // snapshot
  if (this->camera_snapshot && (char) cv::waitKey(1000) == 's') {
    snapshot(image, pose_estimates);
  }

  return pose_estimates;
}

}  // end of awesomo namespace
