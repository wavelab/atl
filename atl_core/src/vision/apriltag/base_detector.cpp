#include "atl/vision/apriltag/base_detector.hpp"

namespace atl {

int BaseDetector::configure(const std::string &config_file) {
  Camera camera;
  ConfigParser parser;
  std::vector<int> tag_ids;
  std::vector<float> tag_sizes;
  std::string config_dir, camera_config;

  // load config
  parser.addParam("tag_ids", &tag_ids);
  parser.addParam("tag_sizes", &tag_sizes);
  parser.addParam("tag_sanity_check", &this->tag_sanity_check);
  parser.addParam("camera_config", &camera_config);
  parser.addParam("illum_invar", &this->illum_invar);
  parser.addParam("windowing", &this->windowing);
  parser.addParam("window_padding", &this->window_padding);
  parser.addParam("imshow", &this->imshow);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // tag detector
  // TO BE CONFIGURED BY DERIVATIVE DETECTORS

  // tag configs
  for (size_t i = 0; i < tag_ids.size(); i++) {
    this->tag_configs[tag_ids[i]] = tag_sizes[i];
  }

  // parse camera configs
  config_dir = std::string(dirname((char *) config_file.c_str()));
  paths_combine(config_dir, camera_config, camera_config);
  if (camera.configure(camera_config) != 0) {
    return -1;
  }
  this->camera_mode = camera.modes[0];
  this->camera_modes = camera.modes;
  this->camera_configs = camera.configs;

  this->configured = true;
  return 0;
}

int BaseDetector::illuminationInvariantTransform(cv::Mat &image) {
  // the following is adapted from:
  // Illumination Invariant Imaging: Applications in Robust Vision-based
  // Localisation, Mapping and Classification for Autonomous Vehicles
  // Maddern et al (2014)
  cv::Mat log_ch_1, log_ch_2, log_ch_3;
  double lambda_1, lambda_2, lambda_3;
  double alpha;
  std::vector<cv::Mat> channels(3);

  lambda_1 = 420;
  lambda_2 = 530;
  lambda_3 = 640;

  // clang-format off
  alpha = (lambda_1 * lambda_3 - lambda_1 * lambda_2) /
          (lambda_2 * lambda_3 - lambda_1 * lambda_2);
  // clang-format on

  split(image, channels);
  channels[0].convertTo(channels[0], CV_32F);
  channels[1].convertTo(channels[1], CV_32F);
  channels[2].convertTo(channels[2], CV_32F);

  channels[0].row(0).setTo(cv::Scalar(1));
  channels[1].row(0).setTo(cv::Scalar(1));
  channels[2].row(0).setTo(cv::Scalar(1));

  cv::log(channels[0] / 255.0, log_ch_1);
  cv::log(channels[1] / 255.0, log_ch_2);
  cv::log(channels[2] / 255.0, log_ch_3);

  image = 0.5 + log_ch_2 - alpha * log_ch_3 - (1 - alpha) * log_ch_1;
  image.setTo(0, image < 0);
  image.setTo(1, image > 1);
  cv::normalize(image, image, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  return 0;
}

int BaseDetector::changeMode(const cv::Mat &image) {
  const int image_width = image.cols;
  const int image_height = image.rows;

  // traverse all camera modes and change mode based on image size
  for (size_t i = 0; i < this->camera_modes.size(); i++) {
    const CameraConfig config = this->camera_configs[this->camera_modes[i]];
    const bool widths_equal = (config.image_width == image_width);
    const bool heights_equal = (config.image_height == image_height);

    if (widths_equal && heights_equal) {
      this->camera_mode = this->camera_modes[i];
      return 1;
    }
  }

  return 0;
}

int BaseDetector::getCameraIntrinsics(double *fx,
                                      double *fy,
                                      double *px,
                                      double *py,
                                      double *image_width,
                                      double *image_height) {
  const std::string camera_mode = this->camera_mode;
  const CameraConfig camera_config = this->camera_configs.at(camera_mode);

  *fx = camera_config.camera_matrix.at<double>(0, 0);
  *fy = camera_config.camera_matrix.at<double>(1, 1);
  *px = camera_config.camera_matrix.at<double>(0, 2);
  *py = camera_config.camera_matrix.at<double>(1, 2);
  *image_width = this->camera_configs[camera_mode].image_width;
  *image_height = this->camera_configs[camera_mode].image_height;

  return 0;
}

int BaseDetector::getTagSize(const TagPose &tag_pose, double *tag_size) {
  if (this->tag_configs.find(tag_pose.id) == this->tag_configs.end()) {
    return -1;
  }
  *tag_size = this->tag_configs[tag_pose.id];

  return 0;
}

int BaseDetector::calculateTagCorners(const cv::Mat &image,
                                      const TagPose &tag_pose,
                                      const double padding,
                                      Vec2 &top_left,
                                      Vec2 &btm_right) {
  // get tag size according to tag id
  double tag_size;
  if (this->getTagSize(prev_tag, &tag_size) != 0) {
    return -1;
  }

  // camera intrinsics
  double fx, fy, px, py, image_width, image_height;
  if (this->getCameraIntrinsics(&fx,
                                &fy,
                                &px,
                                &py,
                                &image_width,
                                &image_height) != 0) {
    return -2;
  }

  // check input image dimensions against configuration file's
  if (image_width != image.cols || image_height != image.rows) {
    LOG_ERROR("Expected image vs input image mismatch!");
    return -3;
  }

  // tag pose is in camera frame
  // camera frame:  (z - forward, x - right, y - down)
  const double x = tag_pose.position(0);
  const double y = tag_pose.position(1);
  const double z = tag_pose.position(2);

  // calculate top left and bottom right corners of tag in inertial frame
  top_left(0) = x - (tag_size / 2.0) - padding;
  top_left(1) = y - (tag_size / 2.0) - padding;
  btm_right(0) = x + (tag_size / 2.0) + padding;
  btm_right(1) = y + (tag_size / 2.0) + padding;

  // project back to image frame (what it would look like in image)
  top_left(0) = (fx * top_left(0) / z) + px;
  top_left(1) = (fy * top_left(1) / z) + py;
  btm_right(0) = (fx * btm_right(0) / z) + px;
  btm_right(1) = (fy * btm_right(1) / z) + py;

  // check corner bounds
  top_left(0) = (top_left(0) > image.cols) ? image.cols : top_left(0);
  top_left(1) = (top_left(1) > image.rows) ? image.rows : top_left(1);
  top_left(0) = (top_left(0) < 0) ? 0 : top_left(0);
  top_left(1) = (top_left(1) < 0) ? 0 : top_left(1);

  btm_right(0) = (btm_right(0) > image.cols) ? image.cols : btm_right(0);
  btm_right(1) = (btm_right(1) > image.rows) ? image.rows : btm_right(1);

  return 0;
}

int BaseDetector::maskImage(const TagPose &prev_tag,
                            const cv::Mat &image,
                            const double padding) {
  // pre-check
  if (image.cols != this->prev_tag_image_width) {
    this->prev_tag.detected = false;
    return -1;
  } else if (image.rows != this->prev_tag_image_height) {
    this->prev_tag.detected = false;
    return -1;
  }

  // calculate tag corners
  Vec2 p1, p2;
  int retval = this->calculateTagCorners(image, prev_tag, padding, p1, p2);
  if (retval != 0) {
    LOG_ERROR("Failed to calculate tag corners!");
    return retval;
  }

  // create mask
  cv::Point top_left(p1(0), p1(1));
  cv::Point bottom_right(p2(0), p2(1));
  cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
  cv::rectangle(mask,
                top_left,
                bottom_right,
                cv::Scalar(255, 255, 255),
                CV_FILLED);

  // mask image
  cv::Mat masked;
  image.copyTo(masked, mask);
  masked.copyTo(image);
  this->prev_tag.detected = false; // reset previous tag

  return 0;
}

int BaseDetector::cropImage(const TagPose &prev_tag,
                            const cv::Mat &image,
                            cv::Mat &cropped_image,
                            const double padding) {
  // calculate tag corners
  Vec2 p1, p2;
  int retval = this->calculateTagCorners(image, prev_tag, padding, p1, p2);
  if (retval != 0) {
    LOG_ERROR("Failed to calculate tag corners!");
    return retval;
  }

  // crop image
  this->crop_x = p1(0);
  this->crop_y = p1(1);
  this->crop_width = p2(0) - p1(0);
  this->crop_height = p2(1) - p1(1);
  cropped_image = image(cv::Rect(this->crop_x,
                                 this->crop_y,
                                 this->crop_width,
                                 this->crop_height));
  this->image_cropped = true;

  return 0;
}

int BaseDetector::preprocessImage(const cv::Mat &image, cv::Mat &image_processed) {
  // change mode based on image size
  this->changeMode(image);

  // tranform illumination invariant tag
  if (this->illum_invar) {
    this->illuminationInvariantTransform(image);
  }

  // mask image if tag was last detected
  cv::Mat cropped_image;
  if (this->prev_tag.detected && this->windowing) {
    // this->maskImage(this->prev_tag, image, this->window_padding);
    this->cropImage(this->prev_tag, image, cropped_image, this->window_padding);
  } else {
    this->crop_x = 0;
    this->crop_y = 0;
    this->crop_width = 0;
    this->crop_height = 0;
    cropped_image = image;
  }
  this->prev_tag.detected = false; // reset previous tag

  // convert image to gray-scale
  cv::Mat image_gray;
  if (image.channels() == 3) {
    cv::cvtColor(cropped_image, image_gray, cv::COLOR_BGR2GRAY);
  } else {
    image_gray = cropped_image;
  }


	return 0;
}

int BaseDetector::getRelativePose(const Vec2 &p1,
                                  const Vec2 &p2,
                                  const Vec2 &p3,
                                  const Vec2 &p4,
                                  TagPose &tag_pose) {
  // get tag size
  double tag_size = 0.0;
  if (this->getTagSize(tag_pose, &tag_size) != 0) {
    return -1;
  }

  // object points
  std::vector<cv::Point3f> obj_pts;
  tag_size = tag_size / 2.0;
  obj_pts.emplace_back(-tag_size, -tag_size, 0);
  obj_pts.emplace_back(tag_size, -tag_size, 0);
  obj_pts.emplace_back(tag_size, tag_size, 0);
  obj_pts.emplace_back(-tag_size, tag_size, 0);

  // image points
  std::vector<cv::Point2f> img_pts;
  if (this->image_cropped) {
    img_pts.emplace_back(p1(0) + this->crop_x, p1(1) + this->crop_y);
    img_pts.emplace_back(p2(0) + this->crop_x, p2(1) + this->crop_y);
    img_pts.emplace_back(p3(0) + this->crop_x, p3(1) + this->crop_y);
    img_pts.emplace_back(p4(0) + this->crop_x, p4(1) + this->crop_y);
    this->image_cropped = false;
  } else {
    img_pts.emplace_back(p1(0), p1(1));
    img_pts.emplace_back(p2(0), p2(1));
    img_pts.emplace_back(p3(0), p3(1));
    img_pts.emplace_back(p4(0), p4(1));
  }

  // solve pnp
  cv::Mat rvec, tvec;
  cv::Vec4f distortion_params(0, 0, 0, 0); // all 0?
  const CameraConfig camera_config = this->camera_configs[this->camera_mode];
  cv::solvePnP(obj_pts,
               img_pts,
               camera_config.camera_matrix,
               distortion_params,
               rvec,
               tvec,
               false,
               CV_ITERATIVE);

  // convert Rodrigues rotation vector to rotation matrix
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);

  // rotation matrix
  // clang-format off
  Mat3 R;
  R << r(0,0), r(0,1), r(0,2),
       r(1,0), r(1,1), r(1,2),
       r(2,0), r(2,1), r(2,2);
  // clang-format on

  // translation
  Vec3 t{tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};

  // tag pose in camera frame
  // camera frame:  (z - forward, x - right, y - down)
  tag_pose.detected = true;
  tag_pose.position = t;
  tag_pose.orientation = Quaternion{R};

  return 0;
}

void BaseDetector::printTag(const TagPose &tag) {
  std::cout << "id: " << tag.id << " ";
  std::cout << "[";
  std::cout << "x= " << tag.position(0) << " ";
  std::cout << "y= " << tag.position(1) << " ";
  std::cout << "z= " << tag.position(2);
  std::cout << "]";
  std::cout << std::endl;
}

} // namespace atl
