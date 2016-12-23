#include "awesomo_core/vision/camera.hpp"


namespace awesomo {

CameraConfig::CameraConfig(int camera_index, int camera_type) {
  this->index = camera_index;
  this->type = camera_type;
  this->snapshot = 0;
  this->exposure_value = 0;
  this->tag_detector = new Detector();

  this->gimbal = NULL;
  this->tag_estimator_initialized = false;
}

CameraConfig::config(std::string config_file) {
  // int retval;
  // float dist;
  // int nb_configs;
  // int nb_apriltags;
  // int apriltag_id;
  // float apriltag_size;
  // std::string config_key;
  // std::string config_path;
  // std::string config_main_path;
  // YAML::Node camera_config;
  //
  // // load camera_config yaml file
  // config_path = config_path + "/camera_config.yaml";
  // camera_config = YAML::LoadFile(config_path);
  // nb_configs = camera_config["nb_configs"].as<int>();
  //
  // // camera type
  // if (camera_config["camera_type"]) {
  //   if (camera_config["camera_type"].as<std::string>() == "firefly") {
  //     this->type = CAMERA_FIREFLY;
  //   } else if (camera_config["camera_type"].as<std::string>() == "ximea") {
  //     this->type = CAMERA_XIMEA;
  //   } else {
  //     this->type = CAMERA_NORMAL;
  //   }
  // } else {
  //   log_err("Opps [camera_type] missing in %s\n", config_path.c_str());
  // }
  //
  // // camera index
  // if (camera_config["camera_index"]) {
  //   this->index = camera_config["camera_index"].as<int>();
  // } else {
  //   log_err("Opps [camera_index] missing in %s\n", config_path.c_str());
  // }
  //
  // // camera imshow
  // if (camera_config["camera_imshow"]) {
  //   this->imshow = camera_config["camera_imshow"].as<int>();
  // } else {
  //   log_err("Opps [camera_imshow] missing in %s\n", config_path.c_str());
  // }
  //
  // // camera snapshot
  // if (camera_config["camera_snapshot"]) {
  //   this->snapshot = camera_config["camera_snapshot"].as<int>();
  // } else {
  //   log_err("Opps [camera_snapshot] missing in %s\n", config_path.c_str());
  // }
  //
  // // camera exposure value
  // if (camera_config["camera_exposure_value"]) {
  //   this->exposure_value =
  //   camera_config["camera_exposure_value"].as<float>();
  //   // std::cout << this->exposure_value << std::endl;
  // } else {
  //   log_err("Opps [camera_exposure_value] missing in %s\n",
  //           config_path.c_str());
  // }
  //
  // // camera gain value
  // if (camera_config["camera_gain_value"]) {
  //   this->gain_value = camera_config["camera_gain_value"].as<float>();
  // } else {
  //   log_err("Opps [camera_gain_value] missing in %s\n",
  //   config_path.c_str());
  // }
  //
  // // tag detector
  // if (camera_config["apriltag_imshow"]) {
  //   this->tag_detector =
  //     new Detector(camera_config["apriltag_imshow"].as<int>());
  // } else {
  //   log_err("Opps [apriltag_imshow] missing in %s\n", config_path.c_str());
  // }
  //
  // // load tag parameters
  // nb_apriltags = camera_config["nb_apriltags"].as<int>();
  // for (int i = 0; i < nb_apriltags; i++) {
  //   apriltag_id = camera_config["apriltag_ids"][i].as<int>();
  //   apriltag_size = camera_config["apriltag_sizes"][i].as<float>();
  //   this->tag_detector->tag_sizes[apriltag_id] = apriltag_size;
  // }
  //
  // // load different calibration files
  // for (int i = 0; i < nb_configs; i++) {
  //   config_key = camera_config["config_keys"][i].as<std::string>();
  //   config_path = config_path + "/";
  //   config_path += camera_config["config_files"][i].as<std::string>();
  //
  //   this->loadConfig(config_key, config_path);
  // }
  //
  // // load config distances
  // for (int i = 0; i < (nb_configs - 1); i++) {
  //   dist = camera_config["config_dists"][i].as<float>();
  //   this->config_dists.push_back(dist);
  // }
  //
  // // print config
  // this->print();
  //
  // // start camera with first calibration file
  // retval = this->initCamera(config["config_keys"][0].as<std::string>());
  // if (retval == -1) {
  //   exit(-1);
  // } else {
  //   log_info("Camera is running...\n");
  // }

  return 0;
}

void CameraConfig::print(void) {
  // clang-format off
  std::cout << "index: " << this->index << std::endl;
  std::cout << "type: " << this->type << std::endl;
  std::cout << "imshow: " << this->imshow << std::endl;
  std::cout << "snapshot: " << this->snapshot << std::endl;
  std::cout << "exposure_value: " << this->exposure_value << std::endl;
  std::cout << "gain_value: " << this->gain_value << std::endl;
  std::cout << "mode: " << this->mode << std::endl;
  std::cout << "lambda: " << this->lambda.transpose() << std::endl;
  std::cout << "alpha: " << this->alpha << std::endl;
  // clang-format on
}

int Camera::initialize(void) {
  // setup
  this->capture = cv::VideoCapture(this->index);

  // open
  if (this->capture.isOpened() == 0) {
    log_error("ERROR! Failed to open webcam!\n");
    return -1;

  } else {
    // this->capture.set(CV_CAP_PROP_FRAME_WIDTH, image_width);
    // this->capture.set(CV_CAP_PROP_FRAME_HEIGHT, image_height);
    log_info("Camera initialized!\n");
    return 0;
  }
}

CameraConfig *Camera::loadConfig(std::string mode, std::string calib_file) {
  CameraConfig *config = new CameraConfig();

  // record config ordering
  this->config_keys.push_back(mode);
  this->config_values.push_back(calib_file);

  // load config
  try {
    YAML::Node config = YAML::LoadFile(calib_file);

    // image width
    if (config["image_width"]) {
      config->image_width = config["image_width"].as<int>();
    } else {
      log_error("ERROR! Failed to load image_width\n");
    }

    // image height
    if (config["image_height"]) {
      config->image_height = config["image_height"].as<int>();
    } else {
      log_error("ERROR! Failed to load image_height\n");
    }

    // camera matrix
    if (config["camera_matrix"]) {
      config->camera_matrix = yamlMat2Mat(config["camera_matrix"]);
    } else {
      log_error("ERROR! Failed to load camera_matrix\n");
    }

    // distortion coefficients
    if (config["distortion_coefficients"]) {
      config->distortion_coefficients =
        yamlMat2Mat(config["distortion_coefficients"]);
    } else {
      log_error("ERROR! Failed to load distortion_coefficients\n");
    }

    // rectification matrix
    if (config["rectification_matrix"]) {
      config->rectification_matrix =
        yamlMat2Mat(config["rectification_matrix"]);
    } else {
      log_error("ERROR! Failed to load rectification_matrix\n");
    }

    // projection matrix
    if (config["projection_matrix"]) {
      config->projection_matrix = yamlMat2Mat(config["projection_matrix"]);
      config->projection_matrix_eigen =
        yamlMat2Mat(config["projection_matrix"]);
    } else {
      log_error("ERROR! Failed to load projection_matrix\n");
    }

  } catch (YAML::BadFile &ex) {
    log_error("ERROR! Failed to load calibration file: %s\n",
              calib_file.c_str());
    throw;
  }

  // add to configs
  this->configs[mode] = config;

  return config;
}

int Camera::loadConfig(std::string mode) {
  // pre-check
  if (this->mode == mode) {
    return 0;
  }

  // load config
  if (this->configs.find(mode) != this->configs.end()) {
    this->mode = mode;
    this->config = this->configs.find(mode)->second;
    log_info("Loaded config file [%s]\n", mode.c_str());

  } else {
    log_error("Config file for mode [%s] not found!\n", mode.c_str());
    return -1;
  }

  return 0;
}

int Camera::setLambdas(float lambda_1, float lambda_2, float lambda_3) {
  // alpha is calculated from:
  // Maddern et al, 2013, Illumination invarent imaging
  this->lambda = lambda;
  this->alpha = (lambda(0) * lambda(2) - lambda(0) * lambda(1)) /
                (lambda(1) * lambda(2) - lambda(0) * lambda(1));

  return 0;
}

// void Camera::adjustMode(std::vector<TagPose> &pose_estimates, int &timeout)
// {
//   TagPose pose;
//   std::string config_key;
//   int config_index;
//   int config_length;
//   float config_dist;
//   float tag_dist;
//
//   // pre-check
//   if (this->config_keys.size() == 1) {
//     return;
//   }
//
//   // load default resolution if no apriltag detected
//   if (timeout > 5 && this->mode != this->config_keys.at(0)) {
//     log_info("timeout!!\n");
//     this->loadConfig(this->config_keys.at(0));
//     timeout = 0;
//     return;
//   } else if (pose_estimates.size() == 0) {
//     timeout++;
//     return;
//   }
//
//   // adjust
//   pose = pose_estimates[0];
//   tag_dist = pose.translation[0];
//   config_length = this->config_keys.size();
//
//   // find config mode index
//   for (config_index = 0; config_index < config_length; config_index++) {
//     config_key = this->config_keys.at(config_index);
//     if (this->mode == config_key) {
//       break;
//     }
//   }
//
//   // load appropriate camera configuration
//   // when camera config is currently between largest and smallest config
//   if (config_index > 0 && config_index < this->config_dists.size()) {
//     // load larger camera config
//     if (tag_dist >= this->config_dists.at(config_index - 1)) {
//       this->loadConfig(this->config_keys.at(config_index - 1));
//
//       // load smaller camera config
//     } else if (tag_dist <= this->config_dists.at(config_index)) {
//       this->loadConfig(this->config_keys.at(config_index + 1));
//     }
//
//     // when camera config is already at largest
//   } else if (config_index == 0) {
//     // load smaller camera config
//     if (tag_dist <= this->config_dists.at(config_index)) {
//       this->loadConfig(this->config_keys.at(config_index + 1));
//     }
//
//     // when camera config is already at smallest
//   } else if (config_index == this->config_dists.size()) {
//     // load larger camera config
//     if (tag_dist >= this->config_dists.at(config_index - 1)) {
//       this->loadConfig(this->config_keys.at(config_index - 1));
//     }
//   }
//   timeout = 0;
// }


void Camera::printFPS(double &last_tic, int &frame) {
  frame++;
  if (frame % 10 == 0) {
    double t = time_now();
    std::cout << "\t" << 10.0 / (t - last_tic) << " fps" << std::endl;
    last_tic = t;
  }
}

int Camera::getFrame(cv::Mat &image) {
  this->capture.read(image);
  return 0;
}

// int Camera::run(void) {
//   int timeout;
//   int frame_index;
//   double last_tic;
//   std::vector<TagPose> pose_estimates;
//
//   // setup
//   timeout = 0;
//   frame_index = 0;
//   last_tic = time_now();
//
//   // read capture device
//   while (true) {
//     pose_estimates = this->step(timeout);
//     this->printFPS(last_tic, frame_index);
//   }
//
//   return 0;
// }

// static void show_image(cv::Mat &image, int tags_detected) {
//   int border;
//   cv::Scalar red;
//   cv::Scalar green;
//
//   // setup
//   border = 5;
//   red = cv::Scalar(0, 0, 255);
//   green = cv::Scalar(0, 255, 0);
//
//   // create detection border
//   if (tags_detected) {
//     cv::copyMakeBorder(image,
//                        image,
//                        border,
//                        border,
//                        border,
//                        border,
//                        cv::BORDER_CONSTANT,
//                        green);
//
//   } else {
//     cv::copyMakeBorder(
//       image, image, border, border, border, border, cv::BORDER_CONSTANT,
//       red);
//   }
//
//   // show image
//   cv::imshow("camera", image);
//   cv::waitKey(1);
// }
//
// static void snapshot(cv::Mat &image, std::vector<TagPose> &pose_estimates)
// {
//   TagPose tag;
//
//   std::cout << "Saving a new image" << std::endl;
//   cv::imshow("image capture", image);
//   cv::imwrite("/tmp/image.png", image);
//
//   if (pose_estimates.size()) {
//     tag = pose_estimates[0];
//     std::cout << "x: " << tag.translation[0] << std::endl;
//     std::cout << "y: " << tag.translation[1] << std::endl;
//     std::cout << "z: " << tag.translation[2] << std::endl;
//   }
// }

// static void tag_estimator_init(struct kf *estimator, Eigen::Vector3d tag) {
//   Eigen::VectorXd mu(9);
//
//   mu << tag(0), tag(1), tag(2),  // x, y, z
//     0.0, 0.0, 0.0,               // velocity x, y, z
//     0.0, 0.0, 0.0;               // acceleration x, y, z
//
//   apriltag_kf_setup(estimator, mu);
// }
//
// void Camera::trackTarget(std::vector<TagPose> pose_estimates) {
//   double dt;
//   Eigen::Vector3d tag;
//   Eigen::Vector3d tag_trans;
//
//   if (pose_estimates.size()) {
//     tag << pose_estimates[0].translation(0),
//     pose_estimates[0].translation(1),
//       pose_estimates[0].translation(2);
//     this->gimbal->transformTargetPosition(tag, tag_trans);
//
//     if (this->tag_estimator_initialized == false) {
//       tag_estimator_init(&this->tag_estimator, tag_trans);
//       this->tag_estimator_initialized = true;
//       tic(&this->tag_estimator_last_updated);
//
//     } else {
//       dt = toc(&this->tag_estimator_last_updated);
//       apriltag_kf_estimate(&this->tag_estimator, tag_trans, dt, true);
//       tic(&this->tag_estimator_last_updated);
//     }
//
//     tag(0) = this->tag_estimator.mu(0);
//     tag(1) = this->tag_estimator.mu(1);
//     tag(2) = this->tag_estimator.mu(2);
//     this->gimbal->trackTarget(tag);
//     tic(&this->tag_last_seen);
//
//   } else if (this->tag_estimator_initialized) {
//     if (toc(&this->tag_last_seen) > 2.0) {
//       this->gimbal->setGimbalAngles(0.0, 0.0, 0.0);
//
//     } else {
//       dt = toc(&this->tag_estimator_last_updated);
//       apriltag_kf_estimate(&this->tag_estimator, tag_trans, dt, false);
//       tic(&this->tag_estimator_last_updated);
//
//       tag(0) = this->tag_estimator.mu(0);
//       tag(1) = this->tag_estimator.mu(1);
//       tag(2) = this->tag_estimator.mu(2);
//       this->gimbal->trackTarget(tag);
//     }
//   }
// }

// std::vector<TagPose> Camera::step(int &timeout) {
//   cv::Mat image;
//   std::vector<TagPose> pose_estimates;
//
//   // get frame
//   if (this->getFrame(image) != 0) {
//     return pose_estimates;
//   }
//
//   // get apriltag pose estimates
//   cv::flip(image, image, -1);
//   pose_estimates = this->tag_detector->processImage(
//     this->config->camera_matrix, image, timeout);
//   this->adjustMode(pose_estimates, timeout);
//
//   // track target
//   this->trackTarget(pose_estimates);
//
//   // imshow
//   if (this->imshow && image.empty() == false) {
//     show_image(image, pose_estimates.size());
//   }
//
//   // snapshot
//   if (this->snapshot && (char) cv::waitKey(1000) == 's') {
//     snapshot(image, pose_estimates);
//   }
//
//   return pose_estimates;
// }
//
// std::vector<TagPose> Camera::step(int &timeout, float dt) {
//   cv::Mat image;
//   Eigen::Vector3d tag;
//   Eigen::Vector3d tag_trans;
//   TagPose tag_pose;
//   std::vector<TagPose> pose_estimates;
//
//   // get frame
//   if (this->getFrame(image) != 0) {
//     return pose_estimates;
//   }
//
//   // get apriltag pose estimates
//   // cv::flip(image, image, -1);
//   pose_estimates = this->tag_detector->processImage(
//     this->config->camera_matrix, image, timeout, dt, this->alpha);
//   this->adjustMode(pose_estimates, timeout);
//
//   // track target
//   this->trackTarget(pose_estimates);
//   if (pose_estimates.size()) {
//     tag_pose = pose_estimates[0];
//     tag << tag_pose.translation(0), tag_pose.translation(1),
//       tag_pose.translation(2);
//     this->gimbal->transformTargetPosition(tag, tag_trans);
//
//     pose_estimates.clear();
//     tag_pose.translation = tag_trans;
//     pose_estimates.push_back(tag_pose);
//   }
//
//   // imshow
//   if (this->imshow) {
//     show_image(image, pose_estimates.size());
//   }
//
//   // snapshot
//   if (this->snapshot && (char) cv::waitKey(1000) == 's') {
//     snapshot(image, pose_estimates);
//   }
//
//   return pose_estimates;
// }

}  // end of awesomo namespace
