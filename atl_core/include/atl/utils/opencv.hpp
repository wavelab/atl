#ifndef ATL_UTILS_OPENCV_HPP
#define ATL_UTILS_OPENCV_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace atl {

bool cvMatIsEqual(const cv::Mat m1, const cv::Mat m2);

}  // namespace atl
#endif
