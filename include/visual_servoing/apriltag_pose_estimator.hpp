#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <opencv2/opencv.hpp>

#include <array>
#include <vector>

#include "apriltag/apriltag.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/tag36h11.h"


struct DetectedTagPose
{
    int id = -1;

    std::array<cv::Point2d, 4> corners;
    cv::Point2d center;

    geometry_msgs::msg::Pose pose_camera;
};


class AprilTagPoseEstimator
{
public:
    explicit AprilTagPoseEstimator(double tag_size);
    ~AprilTagPoseEstimator();

    AprilTagPoseEstimator(const AprilTagPoseEstimator &) = delete;
    AprilTagPoseEstimator &operator=(const AprilTagPoseEstimator &) = delete;

    std::vector<DetectedTagPose> detect(
        const cv::Mat &bgr_image,
        double fx,
        double fy,
        double cx,
        double cy
    );

private:
    geometry_msgs::msg::Pose estimateTagPose(
        apriltag_detection_t *detection,
        double fx,
        double fy,
        double cx,
        double cy
    ) const;

    geometry_msgs::msg::Pose convertAprilTagPoseToRosPose(
        const apriltag_pose_t &pose
    ) const;

    apriltag_detector_t *td_ = nullptr;
    apriltag_family_t *tf_ = nullptr;

    double tag_size_ = 0.024;
};
