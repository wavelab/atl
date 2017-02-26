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

  return 0;
}

int MichiganDetector::extractTags(cv::Mat &image, std::vector<TagPose> &tags) {
  int retval;
  TagPose pose;
  cv::Mat image_gray;
  // std::vector<AprilTags::TagDetection> detections;

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
//     if (this->obtainPose(detections[i], pose) == 0) {
//       tags.push_back(pose);
//
//       // keep track of last tag
//       this->prev_tag = pose;
//       this->prev_tag_image_width = image.cols;
//       this->prev_tag_image_height = image.rows;
//
//       // only need 1 tag
//       break;
//     }
  }

  // imshow
  if (this->imshow) {
    cv::imshow("MichiganDetector", image_gray);
    cv::waitKey(1);
  }

  zarray_destroy(detections);
  return 0;
}

// int MichiganDetector::obtainPose(AprilTags::TagDetection tag, TagPose &tag_pose) {
//   Mat4 transform;
//   Vec3 t;
//   Mat3 R;
//   CameraConfig camera_config;
//   double fx, fy, cx, cy, tag_size;
//
//   // setup
//   camera_config = this->camera_configs[this->camera_mode];
//   fx = camera_config.camera_matrix.at<double>(0, 0);
//   fy = camera_config.camera_matrix.at<double>(1, 1);
//   cx = camera_config.camera_matrix.at<double>(0, 2);
//   cy = camera_config.camera_matrix.at<double>(1, 2);
//   tag_size = 0.0;
//
//   // get tag size according to tag id
//   if (this->tag_configs.find(tag.id) == this->tag_configs.end()) {
//     // log_err("ERROR! Tag size for [%d] not configured!", (int) tag.id);
//     return -2;
//   } else {
//     tag_size = this->tag_configs[tag.id];
//   }
//
//   // recovering the relative transform of a tag:
//   transform = tag.getRelativeTransform(tag_size, fx, fy, cx, cy);
//   t = transform.col(3).head(3);
//   R = transform.block(0, 0, 3, 3);
//
//   // sanity check - calculate euclidean distance between prev and current tag
//   if ((t - this->prev_tag.position).norm() > this->tag_sanity_check) {
//     return -1;
//   }
//
//   // tag is in camera frame
//   // camera frame:  (z - forward, x - right, y - down)
//   tag_pose.id = tag.id;
//   tag_pose.detected = true;
//   tag_pose.position = t;
//   tag_pose.orientation = Quaternion(R);
//
//   return 0;
// }

}  // end of awesomo namespace
