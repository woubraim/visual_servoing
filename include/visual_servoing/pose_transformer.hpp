#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>


/**
 * @brief Helper class responsible for transforming poses between TF frames.
 *
 * This class isolates TF lookup and pose transformation logic from the main
 * visual servoing node. It attempts to transform input poses into a configured
 * target frame and reports failure without throwing, which allows the vision
 * pipeline to keep running even when the robot TF tree is not available.
 */
class PoseTransformer
{
public:
    /**
     * @brief Constructs a pose transformer.
     *
     * @param tf_buffer Shared TF buffer used for transform lookup.
     * @param logger Logger used for throttled warning messages.
     * @param clock Clock used by throttled ROS logging.
     * @param target_frame Frame into which poses should be transformed.
     */
    PoseTransformer(
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        std::string target_frame
    );

    /**
     * @brief Attempts to transform a pose into the configured target frame.
     *
     * @param input_pose Pose to transform.
     * @param output_pose Output pose in target_frame_ if successful.
     * @return true if the transform succeeds, false otherwise.
     */
    bool transformToTargetFrame(
        const geometry_msgs::msg::PoseStamped &input_pose,
        geometry_msgs::msg::PoseStamped &output_pose
    ) const;

    /**
     * @brief Returns the configured target frame.
     */
    const std::string &targetFrame() const;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    std::string target_frame_;
};
