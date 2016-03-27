#include "awesomo/apriltag.hpp"


TagDetector::TagDetector(void)
{
    this->detector = new AprilTags::TagDetector(AprilTags::tagCodes16h5);
}

static cv::Rect enlargeROI(cv::Mat& frm, cv::Rect boundingBox, int padding)
{
    cv::Rect returnRect = cv::Rect(
        boundingBox.x - padding, boundingBox.y - padding,
        boundingBox.width + (padding * 2),
        boundingBox.height + (padding * 2)
    );

    // check the size of the roi
    if (returnRect.x < 0) {
        returnRect.x = 0;
    }
    if (returnRect.y < 0) {
        returnRect.y = 0;
    }
    if (returnRect.x + returnRect.width >= frm.cols) {
        returnRect.width = frm.cols-returnRect.x;
    }
    if (returnRect.y + returnRect.height >= frm.rows) {
        returnRect.height = frm.rows - returnRect.y;
    }

    return returnRect;
}

void TagDetector::adjustROI(cv::Mat &image_gray, AprilTags::TagDetection &tag)
{
    float x;
    float y;
    float normdist;
    cv::Point2f p1;
    cv::Point2f p2;

    // calc the roi rect
    p1 = cv::Point2f(tag.p[1].first, tag.p[1].second);
    p2 = cv::Point2f(tag.p[3].first, tag.p[3].second);
    x = tag.cxy.first;
    y = tag.cxy.second;
    normdist = cv::norm(p2 - p1);

    this->roi_rect = cv::Rect(
        x - normdist / 2,
        y - normdist / 2,
        normdist,
        normdist
    );
    this->roi_rect = enlargeROI(image_gray, this->roi_rect, 5);
}

std::vector<TagPose> TagDetector::processImage(
    cv::Mat &camera_matrix,
    cv::Mat &image,
    int &timeout
)
{
    TagPose pose;
    cv::Mat image_gray;
    std::vector<TagPose> pose_estimates;
    vector<AprilTags::TagDetection> apriltags;

    // apriltags detector (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);

    // create a mask and draw the roi_rect
    cv::Mat mask(image_gray.rows, image_gray.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat masked(image_gray.rows, image_gray.cols, CV_8UC1, cv::Scalar(0));
    cv::rectangle(mask, this->roi_rect, 255, -1);
    image_gray.copyTo(masked, mask);

    // extract apriltags and estimate pose
    apriltags = this->detector->extractTags(masked);
    for (int i = 0; i < apriltags.size(); i++) {
        if (apriltags[i].id == 5 || apriltags[i].id == 0) {
            // this->printDetection(this->apriltags[i]);
            pose = this->obtainPose(apriltags[i], camera_matrix);
            pose_estimates.push_back(pose);
            this->adjustROI(image_gray, apriltags[i]);

            // only need 1 tag
            timeout = 0;
            break;
        }
    }

    // enlarge the roi rect so to be the size of the image
    if (apriltags.size() == 0) {
        this->roi_rect = enlargeROI(image_gray, this->roi_rect, 1000);
    }

    // display result
    // cv::imshow("camera", masked);
    // cv::imshow("camera", image_gray);
    // cv::waitKey(1);

    return pose_estimates;
}

static double standardRad(double t)
{
    // normalize angle to be within the interval [-pi,pi].
    if (t >= 0.) {
        t = fmod(t + M_PI, TWOPI) - M_PI;
    } else {
        t = fmod(t - M_PI, -TWOPI) + M_PI;
    }

    return t;
}

static void convertToEuler(
    const Eigen::Matrix3d &wRo,
    double &yaw,
    double &pitch,
    double &roll
)
{
    double c;
    double s;

    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    c = cos(yaw);
    s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

TagPose TagDetector::obtainPose(
    AprilTags::TagDetection &detection,
    cv::Mat camera_matrix
)
{
    TagPose pose;
    double tag_size;
    Eigen::Matrix3d F;
    Eigen::Matrix3d rotation;
    Eigen::Matrix3d fixed_rot;

    // setup
    F << 1, 0, 0,
         0, -1, 0,
         0, 0, 1;

    // change tag size according to tag id
    if (detection.id == 0) {
        tag_size = 0.048;
    } else if (detection.id == 5) {
        tag_size = 0.343;
    }

    // recovering the relative pose of a tag:
    detection.getRelativeTranslationRotation(
        tag_size,
        camera_matrix.at<double>(0, 0),
        camera_matrix.at<double>(1, 1),
        camera_matrix.at<double>(0, 2),
        camera_matrix.at<double>(1, 2),
        pose.translation,
        rotation
    );
    fixed_rot = F * rotation;
    pose.distance = pose.translation.norm();
    convertToEuler(fixed_rot, pose.yaw, pose.pitch, pose.roll);

    return pose;
}

// void TagDetector::printDetection(AprilTags::TagDetection& tag)
// {
//     TagPose pose;
//
//     pose = this->obtainPose(tag);
//     ROS_INFO("id: %d ", tag.id);
//     ROS_INFO("Hamming: %d ", tag.hammingDistance);
//     ROS_INFO("distance= %fm ", pose.translation.norm());
//     ROS_INFO("x=%f ", pose.translation(0));
//     ROS_INFO("y=%f ", pose.translation(1));
//     ROS_INFO("z=%f ", pose.translation(2));
//     ROS_INFO("yaw=%f ", rad2deg(pose.yaw));
//     ROS_INFO("pitch=%f ", rad2deg(pose.pitch));
//     ROS_INFO("roll=%f \n", rad2deg(pose.roll));
//
//     // also note that for SLAM/multi-view application it is better to
//     // use reprojection error of corner points, because the noise in
//     // this relative pose is very non-Gaussian; see iSAM source code
//     // for suitable factors.
// }
