/**
 * @file pose_save_client.cpp
 * @brief Implements the SAVE POSE service client logic.
 */

#include "visual_servoing/pose_save_client.hpp"


PoseSaveClient::PoseSaveClient(
    rclcpp::Node *node,
    const std::string &service_name
)
    : node_(node)
{
    if (!node_)
    {
        throw std::runtime_error("PoseSaveClient requires a valid ROS node pointer");
    }

    client_ = node_->create_client<visual_servoing::srv::SaveCurrentTagGoal>(
        service_name
    );
}


void PoseSaveClient::resetVisibleTags()
{
    std::lock_guard<std::mutex> lock(visible_tags_mutex_);
    visible_tag_ids_.clear();
}


void PoseSaveClient::addVisibleTag(int tag_id)
{
    std::lock_guard<std::mutex> lock(visible_tags_mutex_);
    visible_tag_ids_.push_back(tag_id);
}


void PoseSaveClient::triggerSave()
{
    if (!client_ || !client_->service_is_ready())
    {
        status_ = "Service not ready";

        RCLCPP_WARN(
            node_->get_logger(),
            "Service /visual_servoing/save_current_tag_goal not ready"
        );

        return;
    }

    const int tag_id = getFirstVisibleTagId();

    if (tag_id < 0)
    {
        status_ = "No visible tag";

        RCLCPP_WARN(
            node_->get_logger(),
            "No visible tag to save"
        );

        return;
    }

    auto request =
        std::make_shared<visual_servoing::srv::SaveCurrentTagGoal::Request>();

    request->tag_id = tag_id;
    request->label = "pose";

    status_ = "Saving tag " + std::to_string(tag_id);

    client_->async_send_request(
        request,
        [this, tag_id](
            rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedFuture future
        )
        {
            handleSaveResponse(tag_id, future);
        }
    );
}


const std::string &PoseSaveClient::status() const
{
    return status_;
}


int PoseSaveClient::getFirstVisibleTagId()
{
    std::lock_guard<std::mutex> lock(visible_tags_mutex_);

    if (visible_tag_ids_.empty())
    {
        return -1;
    }

    return visible_tag_ids_.front();
}


void PoseSaveClient::handleSaveResponse(
    int tag_id,
    rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedFuture future
)
{
    try
    {
        const auto response = future.get();

        if (response->success)
        {
            status_ = "Saved tag " + std::to_string(tag_id);

            RCLCPP_INFO(
                node_->get_logger(),
                "Saved pose for tag %d",
                tag_id
            );

            return;
        }

        status_ = "Save failed";

        RCLCPP_WARN(
            node_->get_logger(),
            "Save failed for tag %d: %s",
            tag_id,
            response->message.c_str()
        );
    }
    catch (const std::exception &e)
    {
        status_ = "Service error";

        RCLCPP_ERROR(
            node_->get_logger(),
            "Service exception: %s",
            e.what()
        );
    }
}
