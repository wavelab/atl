#include "atl/utils/opencv.hpp"


namespace atl {

bool cvMatIsEqual(const cv::Mat m1, const cv::Mat m2) {
  cv::Mat diff;

  // pre-check
  if (m1.empty() && m2.empty()) {
    return true;
  }

  // check dimensions
  if (m1.cols != m2.cols) {
    return false;
  } else if (m1.rows != m2.rows) {
    return false;
  } else if (m1.dims != m2.dims) {
    return false;
  }

  // check matrix elements
  cv::compare(m1, m2, diff, cv::CMP_NE);

  return cv::countNonZero(diff) ? false : true;
}

}  // end of atl namespace
