#include "atl/vision/apriltag/michigan.hpp"


namespace atl {

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
  if (this->prev_tag.detected) {
    retval = this->maskImage(this->prev_tag, image);
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
      this->obtainPose(det, pose);
      // only need 1 tag
      break;
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
  CameraConfig camera_config;
  double fx, fy, cx, cy, tag_size, s;
  std::vector<cv::Point3f> obj_pts;
  std::vector<cv::Point2f> img_pts;



  // setup
  camera_config = this->camera_configs[this->camera_mode];
  // fx = camera_config.camera_matrix.at<double>(0, 0);
  // fy = camera_config.camera_matrix.at<double>(1, 1);
  // cx = camera_config.camera_matrix.at<double>(0, 2);
  // cy = camera_config.camera_matrix.at<double>(1, 2);
  tag_size = 0.0;

  // get tag size according to tag id
  // if (this->tag_configs.find(tag.id) == this->tag_configs.end()) {
  //   // log_err("ERROR! Tag size for [%d] not configured!", (int) tag.id);
  //   return -2;
  // } else {
  //   tag_size = this->tag_configs[tag.id] / 2;
  // }
  //
  tag_size = 0.5;

  obj_pts.push_back(cv::Point3f(-tag_size, -tag_size, 0));
  obj_pts.push_back(cv::Point3f(tag_size, -tag_size, 0));
  obj_pts.push_back(cv::Point3f(tag_size, tag_size, 0));
  obj_pts.push_back(cv::Point3f(-tag_size, tag_size, 0));

  img_pts.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));
  img_pts.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));
  img_pts.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));
  img_pts.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));


  // distortion parameters
  cv::Vec4f distParam(0, 0, 0, 0);


  cv::Mat rvec, tvec;
  // recovering the relative transform of a tag:
  cv::solvePnP(obj_pts, img_pts, camera_config.camera_matrix, distParam, rvec, tvec, false, CV_ITERATIVE);

  cv::Matx33d r;
  cv::Rodrigues(rvec, r);

  // Eigen::Matrix3d wRo;
  R << r(0,0), r(0,1), r(0,2),
         r(1,0), r(1,1), r(1,2),
         r(2,0), r(2,1), r(2,2);

  // translational component
  t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

  // sanity check - calculate euclidean distance between prev and current tag
  // if ((t - this->prev_tag.position).norm() > this->tag_sanity_check) {
  //   return -1;
  // }

  // tag is in camera frame
  // camera frame:  (z - forward, x - right, y - down)
  // tag_pose.id = tag.id;
  tag_pose.detected = true;
  tag_pose.position = t;
  tag_pose.orientation = Quaternion(R);

  return 0;
}

}  // end of atl namespace
