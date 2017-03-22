#include "awesomo_core/vision/apriltag/michigan.hpp"


namespace awesomo {

MichiganDetector::MichiganDetector(void) {
  this->detector = NULL;
}

MichiganDetector::~MichiganDetector(void) {
  apriltag_detector_destroy(this->detector);
  tag16h5_destroy(this->family);
}

int MichiganDetector::configure(std::string config_file) {
  if (BaseDetector::configure(config_file) != 0) {
    return -1;
  }

  // tag detector
  this->detector = apriltag_detector_create();
  this->family = tag16h5_create();
  apriltag_detector_add_family(this->detector, this->family);

  //  detector configuration
  this->detector->nthreads = 8;
  this->detector->quad_decimate = 0;
  this->detector->quad_sigma = 0; // bluring
  // this->detector->debug = getopt_get_bool(getopt, "debug");
  this->detector->refine_edges = 0;
  this->detector->refine_decode = 0;
  this->detector->refine_pose = 0;

  return 0;
}

int MichiganDetector::extractTags(cv::Mat &image, std::vector<TagPose> &tags) {
  int retval;
  TagPose pose;
  cv::Mat image_gray;

  // change mode based on image size
  this->changeMode(image);

  // tranform illumination invariant tag
  if (this->illum_invar) {
    this->illuminationInvariantTransform(image);
  }


  // mask image if tag was last detected
  if (this->prev_tag.detected && this->windowing) {
    retval = this->maskImage(this->prev_tag, image, this->window_padding);
    if (retval == -4) {
      return -1;
    }
  }

  // convert image to gray-scale
  if (image.channels() == 3) {
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  } else {
    image_gray = image;
  }

  // extract tags
  image_u8_t im = {
    .width = image_gray.cols,
    .height = image_gray.rows,
    .stride = image_gray.cols,
    .buf = image_gray.data
  };
  zarray_t *detections = apriltag_detector_detect(this->detector, &im);

  // calculate tag pose
  for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      if (this->obtainPose(det, pose) == 0) {
        this->prev_tag = pose;
        this->prev_tag_image_width = image.cols;
        this->prev_tag_image_height = image.rows;

        tags.push_back(pose);
        // only need 1 tag
        break;
      }
  }
  // imshow
  if (this->imshow) {
    cv::imshow("MichiganDetector", image_gray);
    cv::waitKey(1);
  }

  zarray_destroy(detections);
  return 0;
}

int MichiganDetector::obtainPose(apriltag_detection_t *det, TagPose &tag_pose) {
  Mat4 transform;
  Vec3 t;
  Mat3 R;
  double tag_size;
  double  s;

  CameraConfig camera_config;
  cv::Vec4f distortion_param;
  std::vector<cv::Point3f> obj_pts;
  std::vector<cv::Point2f> img_pts;
  cv::Mat rvec;
  cv::Mat tvec;

  // setup
  camera_config = this->camera_configs[this->camera_mode];
  distortion_param(0) = camera_config.distortion_coefficients.at<double>(0, 0);
  distortion_param(1) = camera_config.distortion_coefficients.at<double>(0, 1);
  distortion_param(2) = camera_config.distortion_coefficients.at<double>(0, 2);
  distortion_param(3) = camera_config.distortion_coefficients.at<double>(0, 3);
  tag_size = 0.0;

  // get tag size according to tag id
  if (this->tag_configs.find(det->id) == this->tag_configs.end()) {
    // log_err("ERROR! Tag size for [%d] not configured!", (int) det->id);
    return -2;
  } else {
    tag_size = this->tag_configs[det->id] / 2;
  }

  // object points in real world (location of each corner of the tag)
  obj_pts.push_back(cv::Point3f(-tag_size, -tag_size, 0));
  obj_pts.push_back(cv::Point3f(tag_size, -tag_size, 0));
  obj_pts.push_back(cv::Point3f(tag_size, tag_size, 0));
  obj_pts.push_back(cv::Point3f(-tag_size, tag_size, 0));

  // location of the above points in the image
  img_pts.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));
  img_pts.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));
  img_pts.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));
  img_pts.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));

  // recovering the relative transform of a tag:
  cv::solvePnP(obj_pts, img_pts, camera_config.camera_matrix, distortion_param, rvec, tvec, false, CV_ITERATIVE);

  // get rotation matrix
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);

  // Eigen::Matrix3d wRo;
  R << r(0,0), r(0,1), r(0,2),
       r(1,0), r(1,1), r(1,2),
       r(2,0), r(2,1), r(2,2);

  // translational component
  t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

  // sanity check - calculate euclidean distance between prev and current tag
  if ((t - this->prev_tag.position).norm() > this->tag_sanity_check) {
    return -1;
  }
  t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

  // tag is in camera frame
  // camera frame:  (z - forward, x - right, y - down)
  tag_pose.id = det->id;
  tag_pose.detected = true;
  tag_pose.position = t;
  tag_pose.orientation = Quaternion(R);
  Vec3 euler_out;
  quat2euler(Quaternion(R), 123, euler_out);

  return 0;
}

}  // end of awesomo namespace
