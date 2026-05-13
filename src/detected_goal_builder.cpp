/**
 * @file detected_goal_builder.cpp
 * @brief Builds visual_servoing detected-goal messages.
 */

#include "visual_servoing/detected_goal_builder.hpp"

#include "visual_servoing/msg/detected_goal.hpp"


visual_servoing::msg::DetectedGoalArray DetectedGoalBuilder::createMessage(
    const rclcpp::Time &stamp,
    const std::string &frame_id
)
{
    visual_servoing::msg::DetectedGoalArray message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;

    return message;
}


void DetectedGoalBuilder::addGoal(
    visual_servoing::msg::DetectedGoalArray &message,
    int tag_id,
    const geometry_msgs::msg::Pose &pose,
    const std::string &frame_id
)
{
    visual_servoing::msg::DetectedGoal detected_goal;
    detected_goal.id = tag_id;
    detected_goal.pose = pose;

    message.header.frame_id = frame_id;
    message.goals.push_back(detected_goal);
}
