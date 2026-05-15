#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>
#include <vector>

#include "visual_servoing/srv/save_current_tag_goal.hpp"


/**
 * @brief Manages the SAVE POSE behavior.
 *
 * This class owns the state and ROS service client needed to save the pose of
 * a currently visible AprilTag. It keeps the main visual servoing node focused
 * on image processing and orchestration.
 *
 * Responsibilities:
 * - store the list of visible tag IDs from the latest frame,
 * - choose which visible tag should be saved,
 * - call /visual_servoing/save_current_tag_goal,
 * - keep a user-facing save status string for the OpenCV display.
 */
class PoseSaveClient
{
public:
    /**
     * @brief Constructs a pose save client.
     *
     * @param node ROS node used to create the service client and access logging.
     * @param service_name Name of the save-pose service.
     */
    PoseSaveClient(
        rclcpp::Node *node,
        const std::string &service_name
    );

    /**
     * @brief Clears the list of tags visible in the current frame.
     */
    void resetVisibleTags();

    /**
     * @brief Registers one tag as visible in the current frame.
     *
     * @param tag_id AprilTag ID.
     */
    void addVisibleTag(int tag_id);

    /**
     * @brief Triggers saving the first currently visible tag.
     *
     * If no tag is visible, or if the service is unavailable, the internal
     * status string is updated and no request is sent.
     */
    void triggerSave();

    /**
     * @brief Returns the current save status displayed in the OpenCV window.
     */
    const std::string &status() const;

private:
    /**
     * @brief Returns the first visible tag ID from the latest frame.
     *
     * @return Tag ID if at least one tag is visible, otherwise -1.
     */
    int getFirstVisibleTagId();

    /**
     * @brief Handles the asynchronous response from the save-pose service.
     *
     * @param tag_id Tag ID that was requested for saving.
     * @param future Future containing the service response.
     */
    void handleSaveResponse(
        int tag_id,
        rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedFuture future
    );

    /// Node pointer used for logging and service creation.
    rclcpp::Node *node_ = nullptr;

    /// Client for /visual_servoing/save_current_tag_goal.
    rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedPtr client_;

    /// User-facing status displayed near the SAVE POSE button.
    std::string status_ = "Ready";

    /// IDs of tags detected in the latest processed frame.
    std::vector<int> visible_tag_ids_;

    /// Protects visible_tag_ids_ because mouse events and image callbacks can overlap.
    std::mutex visible_tags_mutex_;
};
