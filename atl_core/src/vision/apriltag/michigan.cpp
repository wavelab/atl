#include "atl/vision/apriltag/michigan.hpp"

namespace atl {

int MichiganDetector::configure(const std::string &config_file) {
  if (BaseDetector::configure(config_file) != 0) {
    return -1;
  }

  // tag detector
  this->detector = apriltag_detector_create();
  this->detector->quad_decimate = 1.0;
  this->detector->nthreads = 4.0;
  this->detector->refine_edges = 1.0;
  this->detector->refine_decode = 1.0;

  this->family = tag16h5_create();
  apriltag_detector_add_family(this->detector, this->family);

  return 0;
}

int MichiganDetector::extractTags(cv::Mat &image, std::vector<TagPose> &tags) {
  // change mode based on image size
  this->changeMode(image);

  // tranform illumination invariant tag
  if (this->illum_invar) {
    this->illuminationInvariantTransform(image);
  }

  // crop image if tag was last detected
  // if (this->prev_tag.detected && this->windowing) {
  //   this->cropImage(this->prev_tag, image, image, this->window_padding);
  // }

  // convert image to gray-scale
  cv::Mat image_gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  } else {
    image_gray = image;
  }

  // detect tags
  image_u8_t im = {.width = image_gray.cols,
                   .height = image_gray.rows,
                   .stride = image_gray.cols,
                   .buf = image_gray.data};
  zarray_t *detections = apriltag_detector_detect(this->detector, &im);

  // calculate tag pose
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *tag;
    zarray_get(detections, i, &tag);

    // std::cout << "tag id: " << tag->id << std::endl;
    // std::cout << "tag goodness: " << tag->goodness << std::endl;
    // std::cout << "tag hamming: " << tag->hamming << std::endl;
    // std::cout << "tag decision margin: " << tag->decision_margin <<
    // std::endl;
    // std::cout << std::endl;

    TagPose pose;
    if (tag->decision_margin > 50.0 && this->obtainPose(tag, pose) == 0) {
      tags.push_back(pose);

      // keep track of last tag
      this->prev_tag = pose;
      this->prev_tag_image_width = image.cols;
      this->prev_tag_image_height = image.rows;

      // only need 1 tag
      break;
    }
  }
  this->image_cropped = false;

  // imshow
  if (this->imshow) {
    cv::imshow("MichiganDetector", image_gray);
    cv::waitKey(0);
  }

  zarray_destroy(detections);
  return 0;
}

int MichiganDetector::obtainPose(apriltag_detection_t *tag, TagPose &tag_pose) {
  double tag_size = 0.0;

  // get tag size according to tag id
  if (this->tag_configs.find(tag->id) == this->tag_configs.end()) {
    LOG_ERROR("ERROR! Tag size for [%d] not configured!", (int) tag->id);
    return -2;
  } else {
    tag_size = this->tag_configs[tag->id] / 2.0;
  }

  // object points
  std::vector<cv::Point3f> obj_pts;
  obj_pts.push_back(cv::Point3f(-tag_size, -tag_size, 0));
  obj_pts.push_back(cv::Point3f(tag_size, -tag_size, 0));
  obj_pts.push_back(cv::Point3f(tag_size, tag_size, 0));
  obj_pts.push_back(cv::Point3f(-tag_size, tag_size, 0));

  // image points
  std::vector<cv::Point2f> img_pts;
  Vec2 p1, p2, p3, p4;
  if (this->image_cropped) {
    // p1(0) = tag->p[0][0] + this->crop_roi.x;
    // p1(1) = tag->p[0][1] + this->crop_roi.y;
    //
    // p2(0) = tag->p[1][0] + this->crop_roi.x;
    // p2(1) = tag->p[1][1] + this->crop_roi.y;
    //
    // p3(0) = tag->p[2][0] + this->crop_roi.x;
    // p3(0) = tag->p[2][1] + this->crop_roi.y;
    //
    // p4(0) = tag->p[3][0] + this->crop_roi.x;
    // p4(1) = tag->p[3][1] + this->crop_roi.y;
    p1(0) = tag->p[0][0];
    p1(1) = tag->p[0][1];

    p2(0) = tag->p[1][0];
    p2(1) = tag->p[1][1];

    p3(0) = tag->p[2][0];
    p3(0) = tag->p[2][1];

    p4(0) = tag->p[3][0];
    p4(1) = tag->p[3][1];

  } else {
    p1(0) = tag->p[0][0];
    p1(1) = tag->p[0][1];

    p2(0) = tag->p[1][0];
    p2(1) = tag->p[1][1];

    p3(0) = tag->p[2][0];
    p3(0) = tag->p[2][1];

    p4(0) = tag->p[3][0];
    p4(1) = tag->p[3][1];
  }
  img_pts.push_back(cv::Point2f(p1(0), p1(1)));
  img_pts.push_back(cv::Point2f(p2(0), p2(1)));
  img_pts.push_back(cv::Point2f(p3(0), p3(1)));
  img_pts.push_back(cv::Point2f(p4(0), p4(1)));

  // distortion parameters
  cv::Vec4f distortion_params(0.0, 0.0, 0.0, 0.0);

  // recovering the relative transform of a tag:
  cv::Mat rvec, tvec;
  CameraConfig camera_config = this->camera_configs[this->camera_mode];
  cv::solvePnP(obj_pts,
               img_pts,
               camera_config.camera_matrix,
               distortion_params,
               rvec,
               tvec,
               false,
               CV_ITERATIVE);

  // converting Rodrigues rotation vector to rotation matrix
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);

  // rotation matrix
  // clang-format off
  Mat3 R;
  R << r(0, 0), r(0, 1), r(0, 2),
       r(1, 0), r(1, 1), r(1, 2),
       r(2, 0), r(2, 1), r(2, 2);
  // clang-format on

  // translational component
  Vec3 t{tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};

  // sanity check - calculate euclidean distance between prev and current tag
  if ((t - this->prev_tag.position).norm() > this->tag_sanity_check) {
    return -1;
  }

  // tag is in camera frame
  // camera frame:  (z - forward, x - right, y - down)
  tag_pose.id = tag->id;
  tag_pose.detected = true;
  tag_pose.position = t;
  tag_pose.orientation = Quaternion{R};

  return 0;
}

} // namespace atl
