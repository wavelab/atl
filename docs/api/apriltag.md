# apriltag.hpp

**Classes**:
- TagPose
- TagDetector



### TagPose

**Attributes**:

- `int id`
- `double distance`
- `double yaw`
- `double pitch`
- `double roll`
- `Eigen::Vector3d translation`



### TagDetector


#### Constructors

    TagDetector(void)
    TagDetector(int apriltag_imshow)


#### Methods

    void adjustROI(cv::Mat &image_gray, AprilTags::TagDetection &tag)

Adjust the Region of Interest (ROI) within `image_gray` with `tag` detected.
This essentially blacks out the whole image apart from the detected Apriltag.
The intuition is to decrease the image area where the apriltag library has to
find the Apriltag.

---

    std::vector<TagPose> processImage
        cv::Mat &camera_matrix,
        cv::Mat &image,
        int &timeout
    );

Process image frame `image` and uses `camera_matrix` to estimate the Apriltags
if any was detected. The `timeout` argument is used to keep track how many
image frames where Apriltags was not detected.

---

    TagPose obtainPose(AprilTags::TagDetection &detection, cv::Mat camera_matrix)

Obtain Apriltag pose from detected tag `detection` and `camera_matrix`.

---

    void printDetection(AprilTags::TagDetection& detection)

Print detected apriltags.

---
