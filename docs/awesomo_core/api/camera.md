# camera.hpp

**Classes**:
- CameraConfig
- Camera



### CameraConfig

This class is mainly used as a data container to store the camera
properties, such as image width, image height, camera matrix, etc.


#### Attributes

- `int camera_mode`
- `int image_width`
- `int image_height`
- `cv::Mat camera_matrix`
- `cv::Mat rectification_matrix`
- `cv::Mat distortion_coefficients`
- `cv::Mat projection_matrix`





### Camera

This class provides the functionalty to connect to any normal OpenCV compatible
camera as well as a PointGrey FireFly camera. In addition to obtaining camera
access it also performs AprilTag detection where the pose estimates are store
in `Camera::pose_estimates`.


#### Attributes

- `vector<AprilTags::TagDetection> apriltags`
- `std::vector<TagPose> pose_estimates`


#### Constructor

- `Camera(int camera_index, int camera_type)`
- `Camera(std::string camera_config_path)`


#### Methods

    int initCamera(std::string camera_mode)

Initialize Camera to be ready to start getting images from the camera. This
method does not get images from camera yet, for that look at `Camera::run()` or
`Camera::step()`.

There are two accepted `camera_mode` values:
- firefly
- normal

---

    CameraConfig *loadConfig(std::string mode, const std::string calib_file)

Load the configuration into the camera class where `mode` is the key or name of
the mode, and `calib_file` is the path to the camera calibration file (yaml)
produced by ROS's `camera_calibration` package.

---

    int loadConfig(std::string camera_mode)

This overloaded method differs from the above method in that this is meant to
be called to change the incoming image size dynamically. Where `camera_mode` is
the key / name used to load the camera calibration file.

---

    int getFrame(cv::Mat &image)

Obtains a single image frame from the camera, the captured image will be set to
`image`. Returns `0` or `-1` to denote success or failure.

---

    int run(void)

Run Camera and attempt to detect any Apriltags. The AprilTags detected will be
recorded in `Camera::apriltags`.

---

    std::vector<TagPose> step(int &timeout)

Step differs to `Camera::run()` where it only obatins a single image frame and
performs apriltag detection on that same image frame and returns a vector of
`TagPose` objects if any apriltag is detected. `timeout` is an integer
denoting the number of image frames where an apriltag was not detected.

---
