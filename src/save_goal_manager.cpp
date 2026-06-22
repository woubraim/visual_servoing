/**
 * @file save_goal_manager.cpp
 * @brief SAVE POSE service server and YAML persistence for visual-servoing goals.
 *
 * Responsibilities:
 * - subscribe to /visual_servoing/detected_goals,
 * - keep the latest visible tag poses expressed in base_frame,
 * - provide /visual_servoing/save_current_tag_goal,
 * - on request, read the current end-effector pose from TF,
 * - compute tag_T_ee = inverse(base_T_tag) * base_T_ee,
 * - append the result to saved_tag_goals.yaml.
 *
 * This node intentionally lives in the visual_servoing package, not inside the
 * ros2_control controller. File I/O, TF lookup, and save-service logic are kept
 * outside the real-time controller path.
 */

#include <array>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <yaml-cpp/yaml.h>

#include "visual_servoing/msg/detected_goal_array.hpp"
#include "visual_servoing/srv/save_current_tag_goal.hpp"
#include "extender_msgs/msg/shared_control_goal.hpp"
#include "extender_msgs/msg/shared_control_goal_array.hpp"

class SavedGoalManagerNode : public rclcpp::Node
{
public:
    SavedGoalManagerNode()
        : Node("save_goal_manager"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        declareParameters();
        loadParameters();
        initializeRosInterfaces();
        loadSavedGoalsFromYaml();

        RCLCPP_INFO(get_logger(), "Saved goal manager ready.");
        RCLCPP_INFO(get_logger(), "Detected goals topic: %s", detected_goals_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Save service: %s", save_service_name_.c_str());
        RCLCPP_INFO(get_logger(), "base_frame: %s", base_frame_.c_str());
        RCLCPP_INFO(get_logger(), "ee_frame: %s", ee_frame_.c_str());
        RCLCPP_INFO(get_logger(), "YAML path: %s", yaml_path_.c_str());
    }

private:
    struct StoredTagPose
    {
        geometry_msgs::msg::Pose pose;
        rclcpp::Time stamp;
        std::string frame_id;
    };

    struct SavedRelativeGoal
    {
        int tag_id = -1;
        std::string label;
        Eigen::Isometry3d T_tag_goal = Eigen::Isometry3d::Identity();
    };

    std::vector<SavedRelativeGoal> saved_goals_;

    rclcpp::Publisher<extender_msgs::msg::SharedControlGoalArray>::SharedPtr dynamic_goals_pub_;
    std::string dynamic_goals_topic_ = "/shared_control/dynamic_goals";

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------

    std::string detected_goals_topic_ = "/visual_servoing/detected_goals";
    std::string save_service_name_ = "/visual_servoing/save_current_tag_goal";
    std::string yaml_path_ = "saved_tag_goals.yaml";
    std::string base_frame_ = "base_link";
    std::string ee_frame_ = "tool0";
    double tf_timeout_sec_ = 0.2;

    // -------------------------------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------------------------------

    rclcpp::Subscription<visual_servoing::msg::DetectedGoalArray>::SharedPtr detected_goals_sub_;
    rclcpp::Service<visual_servoing::srv::SaveCurrentTagGoal>::SharedPtr save_srv_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::mutex latest_tags_mutex_;
    std::unordered_map<int, StoredTagPose> latest_tags_;

    void declareParameters()
    {
        declare_parameter<std::string>("detected_goals_topic", detected_goals_topic_);
        declare_parameter<std::string>("save_service_name", save_service_name_);
        declare_parameter<std::string>("yaml_path", yaml_path_);
        declare_parameter<std::string>("base_frame", base_frame_);
        declare_parameter<std::string>("ee_frame", ee_frame_);
        declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);
        declare_parameter<std::string>("dynamic_goals_topic", dynamic_goals_topic_);
    }

    void loadParameters()
    {
        detected_goals_topic_ = get_parameter("detected_goals_topic").as_string();
        save_service_name_ = get_parameter("save_service_name").as_string();
        yaml_path_ = get_parameter("yaml_path").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        ee_frame_ = get_parameter("ee_frame").as_string();
        tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
        dynamic_goals_topic_ = get_parameter("dynamic_goals_topic").as_string();
    }

    void initializeRosInterfaces()
    {
        detected_goals_sub_ = create_subscription<visual_servoing::msg::DetectedGoalArray>(
            detected_goals_topic_,
            10,
            std::bind(&SavedGoalManagerNode::detectedGoalsCallback, this, std::placeholders::_1));

        save_srv_ = create_service<visual_servoing::srv::SaveCurrentTagGoal>(
            save_service_name_,
            std::bind(
                &SavedGoalManagerNode::saveCurrentTagGoalCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        dynamic_goals_pub_ =
        create_publisher<extender_msgs::msg::SharedControlGoalArray>(
            dynamic_goals_topic_,
            10);
    }

    // -------------------------------------------------------------------------
    // Callbacks
    // -------------------------------------------------------------------------

    /*void detectedGoalsCallback(const visual_servoing::msg::DetectedGoalArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(latest_tags_mutex_);

        // The visual-servoing node publishes currently visible tags. Replacing the
        // map removes stale tags automatically when a tag disappears.
        latest_tags_.clear();

        for (const auto &goal : msg->goals)
        {
            latest_tags_[goal.id] = StoredTagPose{
                goal.pose,
                rclcpp::Time(msg->header.stamp),
                msg->header.frame_id};
        }
    }*/

    void detectedGoalsCallback(const visual_servoing::msg::DetectedGoalArray::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(latest_tags_mutex_);

            for (const auto &goal : msg->goals)
            {
                latest_tags_[goal.id] = StoredTagPose{
                    goal.pose,
                    rclcpp::Time(msg->header.stamp),
                    msg->header.frame_id};
            }
        }

        publishSavedGoals();
    }

    void saveCurrentTagGoalCallback(
        const std::shared_ptr<visual_servoing::srv::SaveCurrentTagGoal::Request> request,
        std::shared_ptr<visual_servoing::srv::SaveCurrentTagGoal::Response> response)
    {
        const int tag_id = request->tag_id;
        const std::string label = request->label.empty() ? "pose" : request->label;

        StoredTagPose tag;
        {
            std::lock_guard<std::mutex> lock(latest_tags_mutex_);

            const auto it = latest_tags_.find(tag_id);
            if (it == latest_tags_.end())
            {
                response->success = false;
                response->message = "Requested tag is not currently detected.";
                return;
            }

            tag = it->second;
        }

        if (tag.frame_id != base_frame_)
        {
            response->success = false;
            response->message =
                "Detected tag frame is '" + tag.frame_id +
                "', but save_goal_manager expects base_frame '" + base_frame_ + "'.";
            return;
        }

        Eigen::Isometry3d T_base_tag;
        if (!poseToIsometry(tag.pose, T_base_tag))
        {
            response->success = false;
            response->message = "Invalid tag pose quaternion.";
            return;
        }

        Eigen::Isometry3d T_base_ee;
        std::string tf_error;
        if (!lookupEndEffectorPose(T_base_ee, tf_error))
        {
            response->success = false;
            response->message = tf_error;
            return;
        }

        // Desired relative pose: end-effector expressed in the tag frame.
        const Eigen::Isometry3d T_tag_ee = T_base_tag.inverse() * T_base_ee;
        const Eigen::Vector3d p_tag_ee = T_tag_ee.translation();

        Eigen::Quaterniond q_tag_ee(T_tag_ee.linear());
        q_tag_ee.normalize();

        response->position[0] = p_tag_ee.x();
        response->position[1] = p_tag_ee.y();
        response->position[2] = p_tag_ee.z();

        response->orientation_wxyz[0] = q_tag_ee.w();
        response->orientation_wxyz[1] = q_tag_ee.x();
        response->orientation_wxyz[2] = q_tag_ee.y();
        response->orientation_wxyz[3] = q_tag_ee.z();

        if (!appendGoalToYaml(tag_id, label, p_tag_ee, q_tag_ee))
        {
            response->success = false;
            response->message = "Pose computed, but YAML save failed.";
            return;
        }

        SavedRelativeGoal saved;
        saved.tag_id = tag_id;
        saved.label = label;
        saved.T_tag_goal = T_tag_ee;

        saved_goals_.push_back(saved);

        publishSavedGoals();

        response->success = true;
        response->message = "Relative tag-to-end-effector pose saved.";

        RCLCPP_INFO(
            get_logger(),
            "Saved tag %d [%s]: tag_T_ee position=[%.4f %.4f %.4f], q_wxyz=[%.4f %.4f %.4f %.4f]",
            tag_id,
            label.c_str(),
            p_tag_ee.x(),
            p_tag_ee.y(),
            p_tag_ee.z(),
            q_tag_ee.w(),
            q_tag_ee.x(),
            q_tag_ee.y(),
            q_tag_ee.z());
    }

    // -------------------------------------------------------------------------
    // Math / TF helpers
    // -------------------------------------------------------------------------

    bool poseToIsometry(
        const geometry_msgs::msg::Pose &pose,
        Eigen::Isometry3d &T) const
    {
        Eigen::Quaterniond q(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z);

        if (q.norm() < 1e-9)
        {
            return false;
        }

        q.normalize();

        T = Eigen::Isometry3d::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = Eigen::Vector3d(
            pose.position.x,
            pose.position.y,
            pose.position.z);

        return true;
    }

    geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d &T) const
    {
        geometry_msgs::msg::Pose pose;

        const Eigen::Vector3d p = T.translation();
        Eigen::Quaterniond q(T.rotation());
        q.normalize();

        pose.position.x = p.x();
        pose.position.y = p.y();
        pose.position.z = p.z();

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        return pose;
    }

    bool transformToIsometry(
        const geometry_msgs::msg::TransformStamped &transform,
        Eigen::Isometry3d &T) const
    {
        Eigen::Quaterniond q(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z);

        if (q.norm() < 1e-9)
        {
            return false;
        }

        q.normalize();

        T = Eigen::Isometry3d::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = Eigen::Vector3d(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        return true;
    }

    bool lookupEndEffectorPose(Eigen::Isometry3d &T_base_ee, std::string &error)
    {
        try
        {
            // Use the latest available TF at the moment the user clicks SAVE.
            const geometry_msgs::msg::TransformStamped tf_base_ee =
                tf_buffer_.lookupTransform(
                    base_frame_,
                    ee_frame_,
                    rclcpp::Time(0),
                    rclcpp::Duration::from_seconds(tf_timeout_sec_));

            if (!transformToIsometry(tf_base_ee, T_base_ee))
            {
                error = "Invalid end-effector TF quaternion.";
                return false;
            }

            return true;
        }
        catch (const std::exception &e)
        {
            error = std::string("TF lookup failed for ") + base_frame_ + " -> " + ee_frame_ + ": " + e.what();
            return false;
        }
    }

    void publishSavedGoals()
    {
        if (!dynamic_goals_pub_)
        {
            return;
        }

        std::unordered_map<int, StoredTagPose> tags_copy;
        {
            std::lock_guard<std::mutex> lock(latest_tags_mutex_);
            tags_copy = latest_tags_;
        }

        extender_msgs::msg::SharedControlGoalArray msg;

        for (size_t i = 0; i < saved_goals_.size(); ++i)
        {
            const auto &saved = saved_goals_[i];

            const auto tag_it = tags_copy.find(saved.tag_id);
            if (tag_it == tags_copy.end())
            {
                continue;
            }

            const StoredTagPose &tag = tag_it->second;

            if (tag.frame_id != base_frame_)
            {
                continue;
            }

            Eigen::Isometry3d T_base_tag;
            if (!poseToIsometry(tag.pose, T_base_tag))
            {
                continue;
            }

            const Eigen::Isometry3d T_base_goal =
                T_base_tag * saved.T_tag_goal;

            extender_msgs::msg::SharedControlGoal goal_msg;
            goal_msg.id = static_cast<int32_t>(1000 + saved.tag_id * 100 + i);
            goal_msg.goal_pose = isometryToPose(T_base_goal);

            msg.goal_array.push_back(goal_msg);
        }

        if (!msg.goal_array.empty())
        {
            dynamic_goals_pub_->publish(msg);
        }
    }

    // -------------------------------------------------------------------------
    // YAML persistence
    // -------------------------------------------------------------------------

    bool loadSavedGoalsFromYaml()
    {
        saved_goals_.clear();

        YAML::Node root;
        try
        {
            root = YAML::LoadFile(yaml_path_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(
                get_logger(),
                "No saved goals YAML found or could not open: %s. Starting with empty saved goals. Error: %s",
                yaml_path_.c_str(),
                e.what());
            return false;
        }

        if (!root || !root.IsSequence())
        {
            RCLCPP_WARN(
                get_logger(),
                "Saved goals YAML is empty or not a top-level list: %s",
                yaml_path_.c_str());
            return true;
        }

        for (const auto &node : root)
        {
            const int tag_id =
                node["tag_id"] ? node["tag_id"].as<int>() : -1;

            const std::string label =
                node["label"] ? node["label"].as<std::string>() : "pose";

            if (!node["position"] || !node["orientation_wxyz"])
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Skipping saved goal: missing position or orientation_wxyz.");
                continue;
            }

            const std::vector<double> position =
                node["position"].as<std::vector<double>>();

            const std::vector<double> orientation_wxyz =
                node["orientation_wxyz"].as<std::vector<double>>();

            if (tag_id < 0 || position.size() != 3 || orientation_wxyz.size() != 4)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Skipping invalid saved goal. tag_id=%d, position size=%zu, orientation size=%zu",
                    tag_id,
                    position.size(),
                    orientation_wxyz.size());
                continue;
            }

            Eigen::Quaterniond q(
                orientation_wxyz[0],
                orientation_wxyz[1],
                orientation_wxyz[2],
                orientation_wxyz[3]);

            if (q.norm() < 1e-9)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Skipping saved goal for tag %d because quaternion norm is zero.",
                    tag_id);
                continue;
            }

            q.normalize();

            Eigen::Isometry3d T_tag_goal = Eigen::Isometry3d::Identity();
            T_tag_goal.linear() = q.toRotationMatrix();
            T_tag_goal.translation() = Eigen::Vector3d(
                position[0],
                position[1],
                position[2]);

            SavedRelativeGoal saved;
            saved.tag_id = tag_id;
            saved.label = label;
            saved.T_tag_goal = T_tag_goal;

            saved_goals_.push_back(saved);
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded %zu saved relative goals from YAML: %s",
            saved_goals_.size(),
            yaml_path_.c_str());

        return true;
    }


    bool appendGoalToYaml(
        int tag_id,
        const std::string &label,
        const Eigen::Vector3d &position,
        const Eigen::Quaterniond &orientation)
    {
        std::ofstream file(yaml_path_, std::ios::app);

        if (!file.is_open())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open YAML file: %s", yaml_path_.c_str());
            return false;
        }

        file << "- tag_id: " << tag_id << "\n";
        file << "  label: \"" << label << "\"\n";
        file << "  frame: \"tag_" << tag_id << "\"\n";
        file << "  saved_transform: \"tag_T_ee\"\n";
        file << "  position: ["
             << position.x() << ", "
             << position.y() << ", "
             << position.z() << "]\n";
        file << "  orientation_wxyz: ["
             << orientation.w() << ", "
             << orientation.x() << ", "
             << orientation.y() << ", "
             << orientation.z() << "]\n";

        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SavedGoalManagerNode>());
    rclcpp::shutdown();
    return 0;
}
