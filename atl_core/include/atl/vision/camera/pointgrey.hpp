#ifndef ATL_CORE_VISION_CAMERA_POINTGREY_HPP
#define ATL_CORE_VISION_CAMERA_POINTGREY_HPP

#include "atl/vision/camera/camera.hpp"
#include <flycapture/FlyCapture2.h>

namespace atl {

class PointGreyCamera : public Camera {
public:
  FlyCapture2::Camera *pointgrey;

  PointGreyCamera() : pointgrey{nullptr} {}
  ~PointGreyCamera();

  /**
   * Initialize camera
   *
   * @return 0 for success, -1 for failure
   */
  int initialize();

  /**
   * Set brightness
   *
   * @param brightness
   * @return 0 for success, -1 for failure
   */
  int setBrightness(double brightness);

  /**
   * Set frame rate
   *
   * @param frame rate
   * @return 0 for success, -1 for failure
   */
  int setFrameRate(double fps);

  /**
   * Set exposure
   *
   * @param exposure
   * @return 0 for success, -1 for failure
   */
  int setExposure(double exposure);

  /**
   * Set shutter
   *
   * @param shutter
   * @return 0 for success, -1 for failure
   */
  int setShutter(double shutter_ms);

  /**
   * Set gain
   *
   * @param gain
   * @return 0 for success, -1 for failure
   */
  int setGain(double gain_db);

  /**
   * Get brightness
   *
   * @param brightness
   * @return 0 for success, -1 for failure
   */
  int getBrightness(double &brightness);

  /**
   * Get frame rate
   *
   * @param frame rate
   * @return 0 for success, -1 for failure
   */
  int getFrameRate(double &fps);

  /**
   * Get exposure
   *
   * @param exposure
   * @return 0 for success, -1 for failure
   */
  int getExposure(double &exposure);

  /**
   * Get shutter speed
   *
   * @param shutter speed
   * @return 0 for success, -1 for failure
   */
  int getShutter(double &shutter_ms);

  /**
   * Get gain
   *
   * @param gain
   * @return 0 for success, -1 for failure
   */
  int getGain(double &gain_db);

  /**
   * Print Format7 capabilities
   *
   * @return 0 for success, -1 for failure
   */
  int printFormat7Capabilities();

  /**
   * Set Format7 settings
   *
   * @param mode Format7 mode
   * @param pixel_format Pixel format
   * @param width Image width
   * @param height Image height
   *
   * @return 0 for success, -1 for failure
   */
  int setFormat7(const int mode,
                 const std::string &pixel_format,
                 const int width,
                 const int height);

  /**
   * Calculate center ROI
   *
   * @param size
   * @param max_size
   * @param step
   *
   * @return Center of ROI
   */
  std::pair<int, int> centerROI(const int size,
                                const int max_size,
                                const int step);

  /**
   * Change mode
   *
   * @param mode Mode
   * @return 0 for success, -1 for failure
   */
  int changeMode(const std::string &mode);

  /**
   * Get frame
   *
   * @param image Image
   * @return 0 for success, -1 for failure
   */
  int getFrame(cv::Mat &image);

  /**
   * Run
   *
   * @return 0 for success, -1 for failure
   */
  int run();
};

} // namespace atl
#endif
