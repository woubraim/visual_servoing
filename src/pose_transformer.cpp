#include "visual_servoing/pose_transformer.hpp"


PoseTransformer::PoseTransformer(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    std::string target_frame
)
    : tf_buffer_(std::move(tf_buffer)),
      logger_(logger),
      clock_(std::move(clock)),
      target_frame_(std::move(target_frame))
{
}


bool PoseTransformer::transformToTargetFrame(
    const geometry_msgs::msg::PoseStamped &input_pose,
    geometry_msgs::msg::PoseStamped &output_pose
) const
{
    try
    {
        const geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(
                target_frame_,
                input_pose.header.frame_id,
                tf2::TimePointZero
            );

        tf2::doTransform(input_pose, output_pose, transform_stamped);
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(
            logger_,
            *clock_,
            2000,
            "Could not transform from %s to %s: %s. Displaying tag in camera image only.",
            input_pose.header.frame_id.c_str(),
            target_frame_.c_str(),
            ex.what()
        );

        return false;
    }
}


const std::string &PoseTransformer::targetFrame() const
{
    return target_frame_;
}
