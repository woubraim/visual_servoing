#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/apriltag_pose.h"

#include "visual_servoing/msg/detected_goal.hpp"
#include "visual_servoing/msg/detected_goal_array.hpp"
#include "visual_servoing/srv/save_current_tag_goal.hpp"


class VisualServoingNode : public rclcpp::Node
{
public:
    VisualServoingNode()
        : Node("visual_servoing_node")
    {
        td_ = apriltag_detector_create();
        tf_ = tag36h11_create();
        apriltag_detector_add_family(td_, tf_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter<std::string>("camera_type", "oak");
        this->declare_parameter<bool>("fullscreen_display", true);
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<double>("tag_size", 0.024);

        camera_type_ = this->get_parameter("camera_type").as_string();
        fullscreen_display_ = this->get_parameter("fullscreen_display").as_bool();
        target_frame_ = this->get_parameter("target_frame").as_string();
        tag_size_ = this->get_parameter("tag_size").as_double();

        if (camera_type_ == "oak")
        {
            image_topic_ = "/oak/rgb/image_raw";
            camera_info_topic_ = "/oak/rgb/camera_info";
            optical_frame_ = "oak_rgb_camera_optical_frame";
            window_name_ = "Visual Servoing OAK";
        }
        else if (camera_type_ == "realsense")
        {
            image_topic_ = "/camera/camera/color/image_raw";
            camera_info_topic_ = "/camera/camera/color/camera_info";
            optical_frame_ = "camera_color_optical_frame";
            window_name_ = "Visual Servoing Realsense";
        }
        else
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Unknown camera_type: '%s'. Use 'oak' or 'realsense'.",
                camera_type_.c_str()
            );

            throw std::runtime_error("Invalid camera_type");
        }

        RCLCPP_INFO(this->get_logger(), "Using camera_type: %s", camera_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera info topic: %s", camera_info_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Optical frame: %s", optical_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Tag size: %.3f m", tag_size_);

        detected_goals_pub_ =
            this->create_publisher<visual_servoing::msg::DetectedGoalArray>(
                "/visual_servoing/detected_goals",
                10
            );

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_,
            10,
            std::bind(&VisualServoingNode::imageCallback, this, std::placeholders::_1)
        );

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_,
            10,
            std::bind(&VisualServoingNode::cameraInfoCallback, this, std::placeholders::_1)
        );

        save_pose_client_ =
            this->create_client<visual_servoing::srv::SaveCurrentTagGoal>(
                "/visual_servoing/save_current_tag_goal"
            );

        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);

        if (fullscreen_display_)
        {
            cv::setWindowProperty(
                window_name_,
                cv::WND_PROP_FULLSCREEN,
                cv::WINDOW_FULLSCREEN
            );
        }

        cv::setMouseCallback(window_name_, &VisualServoingNode::mouseCallback, this);
    }

    ~VisualServoingNode()
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

private:
    std::string camera_type_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string optical_frame_;
    std::string window_name_;

    bool fullscreen_display_ = true;

    std::string target_frame_ = "base_link";
    double tag_size_ = 0.024;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Publisher<visual_servoing::msg::DetectedGoalArray>::SharedPtr detected_goals_pub_;

    rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedPtr save_pose_client_;

    apriltag_detector_t *td_ = nullptr;
    apriltag_family_t *tf_ = nullptr;

    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;

    std::vector<int> visible_tag_ids_;
    std::mutex visible_tags_mutex_;

    cv::Rect save_button_rect_{10, 10, 180, 40};
    std::string save_status_ = "Ready";

    static void mouseCallback(int event, int x, int y, int flags, void *userdata)
    {
        (void)flags;

        if (!userdata)
        {
            return;
        }

        VisualServoingNode *self = static_cast<VisualServoingNode *>(userdata);
        self->handleMouse(event, x, y);
    }

    void handleMouse(int event, int x, int y)
    {
        if (event != cv::EVENT_LBUTTONDOWN)
        {
            return;
        }

        if (save_button_rect_.contains(cv::Point(x, y)))
        {
            triggerSave();
        }
    }

    void triggerSave()
    {
        if (!save_pose_client_ || !save_pose_client_->service_is_ready())
        {
            save_status_ = "Service not ready";
            RCLCPP_WARN(this->get_logger(), "Service /visual_servoing/save_current_tag_goal not ready");
            return;
        }

        int tag_id = -1;

        {
            std::lock_guard<std::mutex> lock(visible_tags_mutex_);

            if (visible_tag_ids_.empty())
            {
                save_status_ = "No visible tag";
                RCLCPP_WARN(this->get_logger(), "No visible tag to save");
                return;
            }

            tag_id = visible_tag_ids_.front();
        }

        auto req =
            std::make_shared<visual_servoing::srv::SaveCurrentTagGoal::Request>();

        req->tag_id = tag_id;
        req->label = "pose";

        save_status_ = "Saving tag " + std::to_string(tag_id);

        save_pose_client_->async_send_request(
            req,
            [this, tag_id](
                rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedFuture future
            )
            {
                try
                {
                    auto res = future.get();

                    if (res->success)
                    {
                        save_status_ = "Saved tag " + std::to_string(tag_id);

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Saved pose for tag %d",
                            tag_id
                        );
                    }
                    else
                    {
                        save_status_ = "Save failed";

                        RCLCPP_WARN(
                            this->get_logger(),
                            "Save failed for tag %d: %s",
                            tag_id,
                            res->message.c_str()
                        );
                    }
                }
                catch (const std::exception &e)
                {
                    save_status_ = "Service error";

                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Service exception: %s",
                        e.what()
                    );
                }
            }
        );
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (fx_ == 0.0 || fy_ == 0.0)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Camera info not received yet!"
            );
            return;
        }

        cv::Mat color;

        try
        {
            color = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray;
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

        image_u8_t im_header = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td_, &im_header);
        const int ndets = zarray_size(detections);

        {
            std::lock_guard<std::mutex> lock(visible_tags_mutex_);
            visible_tag_ids_.clear();
        }

        visual_servoing::msg::DetectedGoalArray detected_goals_msg;
        detected_goals_msg.header.stamp = this->now();
        detected_goals_msg.header.frame_id = target_frame_;

        for (int i = 0; i < ndets; i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = tag_size_;
            info.fx = fx_;
            info.fy = fy_;
            info.cx = cx_;
            info.cy = cy_;

            apriltag_pose_t pose;
            const double err = estimate_tag_pose(&info, &pose);
            (void)err;

            const double x = matd_get(pose.t, 0, 0);
            const double y = matd_get(pose.t, 1, 0);
            const double z = matd_get(pose.t, 2, 0);

            const double r11 = matd_get(pose.R, 0, 0);
            const double r12 = matd_get(pose.R, 0, 1);
            const double r13 = matd_get(pose.R, 0, 2);

            const double r21 = matd_get(pose.R, 1, 0);
            const double r22 = matd_get(pose.R, 1, 1);
            const double r23 = matd_get(pose.R, 1, 2);

            const double r31 = matd_get(pose.R, 2, 0);
            const double r32 = matd_get(pose.R, 2, 1);
            const double r33 = matd_get(pose.R, 2, 2);

            Eigen::Matrix3d R;
            R << r11, r12, r13,
                 r21, r22, r23,
                 r31, r32, r33;

            Eigen::Quaterniond q(R);
            q.normalize();

            geometry_msgs::msg::Pose tag_pose_camera;
            tag_pose_camera.position.x = x;
            tag_pose_camera.position.y = y;
            tag_pose_camera.position.z = z;
            tag_pose_camera.orientation.x = q.x();
            tag_pose_camera.orientation.y = q.y();
            tag_pose_camera.orientation.z = q.z();
            tag_pose_camera.orientation.w = q.w();

            geometry_msgs::msg::PoseStamped tag_pose_camera_stamped;
            tag_pose_camera_stamped.header.stamp = msg->header.stamp;
            tag_pose_camera_stamped.header.frame_id = msg->header.frame_id;
            tag_pose_camera_stamped.pose = tag_pose_camera;

            for (int j = 0; j < 4; j++)
            {
                cv::line(
                    color,
                    cv::Point(det->p[j][0], det->p[j][1]),
                    cv::Point(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]),
                    cv::Scalar(0, 0, 255),
                    2
                );
            }

            cv::putText(
                color,
                std::to_string(det->id),
                cv::Point(static_cast<int>(det->c[0]), static_cast<int>(det->c[1])),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 255, 0),
                2
            );

            try
            {
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform(
                        target_frame_,
                        tag_pose_camera_stamped.header.frame_id,
                        tf2::TimePointZero
                    );

                geometry_msgs::msg::PoseStamped tag_pose_target;
                tf2::doTransform(tag_pose_camera_stamped, tag_pose_target, transform_stamped);

                visual_servoing::msg::DetectedGoal detected_goal;
                detected_goal.id = det->id;
                detected_goal.pose = tag_pose_target.pose;

                detected_goals_msg.goals.push_back(detected_goal);

                {
                    std::lock_guard<std::mutex> lock(visible_tags_mutex_);
                    visible_tag_ids_.push_back(det->id);
                }
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Could not transform from %s to %s: %s. Displaying tag in camera image only.",
                    tag_pose_camera_stamped.header.frame_id.c_str(),
                    target_frame_.c_str(),
                    ex.what()
                );

                // Keep the tag visible/selectable even if TF is missing.
                {
                    std::lock_guard<std::mutex> lock(visible_tags_mutex_);
                    visible_tag_ids_.push_back(det->id);
                }

                // Publish fallback pose in the camera frame.
                visual_servoing::msg::DetectedGoal detected_goal;
                detected_goal.id = det->id;
                detected_goal.pose = tag_pose_camera;

                detected_goals_msg.header.frame_id = tag_pose_camera_stamped.header.frame_id;
                detected_goals_msg.goals.push_back(detected_goal);
            }

        }

        detected_goals_pub_->publish(detected_goals_msg);

        drawSaveButton(color);

        cv::imshow(window_name_, color);
        cv::waitKey(1);

        apriltag_detections_destroy(detections);
    }

    void drawSaveButton(cv::Mat &image)
    {
        cv::rectangle(
            image,
            save_button_rect_,
            cv::Scalar(255, 0, 0),
            cv::FILLED
        );

        cv::putText(
            image,
            "SAVE POSE",
            cv::Point(save_button_rect_.x + 15, save_button_rect_.y + 27),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(255, 255, 255),
            2
        );

        cv::putText(
            image,
            save_status_,
            cv::Point(10, 65),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 255),
            2
        );
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<VisualServoingNode>());

    rclcpp::shutdown();

    return 0;
}