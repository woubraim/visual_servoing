/**
 * @file shared_control_visualization_node.cpp
 * @brief Publishes RViz markers for visual servoing and shared-control debug data.
 *
 * This node converts internal goal/debug messages into visualization markers.
 * RViz can display MarkerArray messages, but it cannot directly display custom
 * goal messages such as visual_servoing::msg::DetectedGoalArray.
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "visual_servoing/msg/detected_goal_array.hpp"


class SharedControlVisualizationNode : public rclcpp::Node
{
public:
    SharedControlVisualizationNode()
        : Node("shared_control_visualization_node")
    {
        declareParameters();
        loadParameters();
        initializePublishers();
        initializeSubscribers();

        RCLCPP_INFO(get_logger(), "Shared control visualization node ready.");
    }

private:
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------

    double goal_sphere_diameter_ = 0.04;
    double goal_label_height_ = 0.04;
    double marker_lifetime_sec_ = 0.2;

    // -------------------------------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------------------------------

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<visual_servoing::msg::DetectedGoalArray>::SharedPtr
        detected_goals_sub_;

    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    void declareParameters()
    {
        declare_parameter<double>("goal_sphere_diameter", goal_sphere_diameter_);
        declare_parameter<double>("goal_label_height", goal_label_height_);
        declare_parameter<double>("marker_lifetime_sec", marker_lifetime_sec_);
    }

    void loadParameters()
    {
        goal_sphere_diameter_ =
            get_parameter("goal_sphere_diameter").as_double();

        goal_label_height_ =
            get_parameter("goal_label_height").as_double();

        marker_lifetime_sec_ =
            get_parameter("marker_lifetime_sec").as_double();
    }

    void initializePublishers()
    {
        marker_pub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(
                "/shared_control/visualization_markers",
                10
            );
    }

    void initializeSubscribers()
    {
        detected_goals_sub_ =
            create_subscription<visual_servoing::msg::DetectedGoalArray>(
                "/visual_servoing/detected_goals",
                10,
                std::bind(
                    &SharedControlVisualizationNode::detectedGoalsCallback,
                    this,
                    std::placeholders::_1
                )
            );
    }

    // -------------------------------------------------------------------------
    // Callbacks
    // -------------------------------------------------------------------------

    void detectedGoalsCallback(
        const visual_servoing::msg::DetectedGoalArray::SharedPtr msg
    )
    {
        visualization_msgs::msg::MarkerArray markers;

        addClearMarker(markers, msg->header);

        int marker_id = 0;

        for (const auto &goal : msg->goals)
        {
            addGoalSphere(markers, msg->header, goal.id, goal.pose, marker_id++);
            addGoalLabel(markers, msg->header, goal.id, goal.pose, marker_id++);
        }

        marker_pub_->publish(markers);
    }

    // -------------------------------------------------------------------------
    // Marker helpers
    // -------------------------------------------------------------------------

    void addClearMarker(
        visualization_msgs::msg::MarkerArray &markers,
        const std_msgs::msg::Header &header
    ) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header = header;
        marker.action = visualization_msgs::msg::Marker::DELETEALL;

        markers.markers.push_back(marker);
    }

    void addGoalSphere(
        visualization_msgs::msg::MarkerArray &markers,
        const std_msgs::msg::Header &header,
        int tag_id,
        const geometry_msgs::msg::Pose &pose,
        int marker_id
    ) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header = header;
        marker.ns = "detected_goals";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = pose;

        marker.scale.x = goal_sphere_diameter_;
        marker.scale.y = goal_sphere_diameter_;
        marker.scale.z = goal_sphere_diameter_;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);

        (void)tag_id;

        markers.markers.push_back(marker);
    }

    void addGoalLabel(
        visualization_msgs::msg::MarkerArray &markers,
        const std_msgs::msg::Header &header,
        int tag_id,
        const geometry_msgs::msg::Pose &pose,
        int marker_id
    ) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header = header;
        marker.ns = "detected_goal_labels";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = pose;
        marker.pose.position.z += 0.06;

        marker.scale.z = goal_label_height_;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.text = "tag " + std::to_string(tag_id);
        marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);

        markers.markers.push_back(marker);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SharedControlVisualizationNode>());

    rclcpp::shutdown();

    return 0;
}
