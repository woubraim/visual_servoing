/**
 * @file apriltag_pose_estimator.cpp
 * @brief Wraps AprilTag detection and pose estimation.
 *
 * This file isolates the C AprilTag API from the ROS node. It converts OpenCV
 * images into the format expected by the AprilTag detector, extracts tag corners,
 * estimates each tag pose using camera intrinsics, and converts the result into
 * ROS geometry_msgs::msg::Pose.
 */

#include "visual_servoing/apriltag_pose_estimator.hpp"

#include <Eigen/Dense>

#include <stdexcept>


/**
 * @brief Creates the AprilTag detector and registers the tag36h11 family.
 */
AprilTagPoseEstimator::AprilTagPoseEstimator(double tag_size)
    : tag_size_(tag_size)
{
    td_ = apriltag_detector_create();
    tf_ = tag36h11_create();

    if (!td_ || !tf_)
    {
        throw std::runtime_error("Failed to initialize AprilTag detector");
    }

    apriltag_detector_add_family(td_, tf_);
}


/**
 * @brief Releases the AprilTag detector and tag family allocated by the C API.
 */
AprilTagPoseEstimator::~AprilTagPoseEstimator()
{
    if (td_)
    {
        apriltag_detector_destroy(td_);
    }

    if (tf_)
    {
        tag36h11_destroy(tf_);
    }
}

/**
 * @brief Detects AprilTags in a BGR image and estimates their poses.
 *
 * @param bgr_image Input OpenCV image in BGR format.
 * @param fx Camera focal length in pixels along x.
 * @param fy Camera focal length in pixels along y.
 * @param cx Camera principal point x coordinate.
 * @param cy Camera principal point y coordinate.
 * @return List of detected tags with image corners, center, ID, and camera-frame pose.
 */
std::vector<DetectedTagPose> AprilTagPoseEstimator::detect(
    const cv::Mat &bgr_image,
    double fx,
    double fy,
    double cx,
    double cy
)
{
    std::vector<DetectedTagPose> detected_tags;

    cv::Mat gray;
    cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);

    image_u8_t image_header = {
        .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data
    };

    zarray_t *detections = apriltag_detector_detect(td_, &image_header);

    if (!detections)
    {
        return detected_tags;
    }

    const int number_of_detections = zarray_size(detections);

    detected_tags.reserve(number_of_detections);

    for (int i = 0; i < number_of_detections; ++i)
    {
        apriltag_detection_t *detection = nullptr;
        zarray_get(detections, i, &detection);

        if (!detection)
        {
            continue;
        }

        DetectedTagPose detected_tag;
        detected_tag.id = detection->id;

        for (int corner_index = 0; corner_index < 4; ++corner_index)
        {
            detected_tag.corners[corner_index] = cv::Point2d(
                detection->p[corner_index][0],
                detection->p[corner_index][1]
            );
        }

        detected_tag.center = cv::Point2d(
            detection->c[0],
            detection->c[1]
        );

        detected_tag.pose_camera = estimateTagPose(
            detection,
            fx,
            fy,
            cx,
            cy
        );

        detected_tags.push_back(detected_tag);
    }

    apriltag_detections_destroy(detections);

    return detected_tags;
}

/**
 * @brief Estimates the 3D pose of a single AprilTag in the camera frame.
 */
geometry_msgs::msg::Pose AprilTagPoseEstimator::estimateTagPose(
    apriltag_detection_t *detection,
    double fx,
    double fy,
    double cx,
    double cy
) const
{
    apriltag_detection_info_t info;
    info.det = detection;
    info.tagsize = tag_size_;
    info.fx = fx;
    info.fy = fy;
    info.cx = cx;
    info.cy = cy;

    apriltag_pose_t pose;
    const double error = estimate_tag_pose(&info, &pose);
    (void)error;

    return convertAprilTagPoseToRosPose(pose);
}

/**
 * @brief Converts the AprilTag C pose representation into a ROS Pose message.
 */
geometry_msgs::msg::Pose AprilTagPoseEstimator::convertAprilTagPoseToRosPose(
    const apriltag_pose_t &pose
) const
{
    const double x = matd_get(pose.t, 0, 0);
    const double y = matd_get(pose.t, 1, 0);
    const double z = matd_get(pose.t, 2, 0);

    Eigen::Matrix3d rotation;
    rotation <<
        matd_get(pose.R, 0, 0), matd_get(pose.R, 0, 1), matd_get(pose.R, 0, 2),
        matd_get(pose.R, 1, 0), matd_get(pose.R, 1, 1), matd_get(pose.R, 1, 2),
        matd_get(pose.R, 2, 0), matd_get(pose.R, 2, 1), matd_get(pose.R, 2, 2);

    Eigen::Quaterniond quaternion(rotation);
    quaternion.normalize();

    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = x;
    ros_pose.position.y = y;
    ros_pose.position.z = z;
    ros_pose.orientation.x = quaternion.x();
    ros_pose.orientation.y = quaternion.y();
    ros_pose.orientation.z = quaternion.z();
    ros_pose.orientation.w = quaternion.w();

    return ros_pose;
}
