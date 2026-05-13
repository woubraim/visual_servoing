#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <string>

#include "visual_servoing/msg/detected_goal_array.hpp"


/**
 * @brief Utility class used to build DetectedGoalArray messages.
 *
 * This class centralizes message construction for detected visual servoing goals.
 * It keeps ROS message formatting out of the main node logic.
 */
class DetectedGoalBuilder
{
public:
    /**
     * @brief Creates an empty DetectedGoalArray message.
     *
     * @param stamp Timestamp to use in the message header.
     * @param frame_id Frame in which the poses are expressed.
     * @return Initialized DetectedGoalArray message.
     */
    static visual_servoing::msg::DetectedGoalArray createMessage(
        const rclcpp::Time &stamp,
        const std::string &frame_id
    );

    /**
     * @brief Adds one detected goal to an existing DetectedGoalArray message.
     *
     * @param message Message to update.
     * @param tag_id AprilTag ID.
     * @param pose Pose associated with the detected tag.
     * @param frame_id Frame in which the pose is expressed.
     *
     * @note DetectedGoalArray currently stores one shared header.frame_id.
     * Therefore, all poses in the message are expected to share the same frame.
     * If mixed frames are needed later, DetectedGoal.msg should include its own
     * frame_id per goal.
     */
    static void addGoal(
        visual_servoing::msg::DetectedGoalArray &message,
        int tag_id,
        const geometry_msgs::msg::Pose &pose,
        const std::string &frame_id
    );
};
