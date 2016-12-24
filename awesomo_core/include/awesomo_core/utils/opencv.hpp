#ifndef __AWESOMO_UTILS_OPENCV_HPP__
#define __AWESOMO_UTILS_OPENCV_HPP__

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace awesomo {

bool cvMatIsEqual(const cv::Mat m1, const cv::Mat m2);

}  // end of awesomo namespace
#endif
