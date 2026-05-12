#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "apriltag/apriltag.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/tag36h11.h"

#include "visual_servoing/visual_servoing_display.hpp"

#include "visual_servoing/msg/detected_goal.hpp"
#include "visual_servoing/msg/detected_goal_array.hpp"
#include "visual_servoing/srv/save_current_tag_goal.hpp"


// -----------------------------------------------------------------------------
// VisualServoingNode
//
// This node detects AprilTags from a camera image, estimates their 3D pose,
// optionally transforms each pose into a target robot frame, and publishes the
// detected goals.
//
// Important behavior:
// - If the TF transform is available, the tag pose is published in target_frame_.
// - If the TF transform is missing, the tag is still displayed and its pose is
//   published in the camera frame. This makes camera-only testing possible even
//   before the full robot TF tree is running.
// - The OpenCV visualization logic is isolated in VisualServoingDisplay.
// -----------------------------------------------------------------------------
class VisualServoingNode : public rclcpp::Node
{
public:
    VisualServoingNode()
        : Node("visual_servoing_node")
    {
        initializeAprilTagDetector();
        initializeTf();
        loadParameters();
        configureCameraTopics();
        logConfiguration();
        initializeRosInterfaces();
        initializeDisplay();
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
    // -------------------------------------------------------------------------
    // Runtime configuration loaded from ROS parameters
    // -------------------------------------------------------------------------
    std::string camera_type_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string optical_frame_;
    std::string window_name_;

    bool fullscreen_display_ = true;

    std::string target_frame_ = "base_link";
    double tag_size_ = 0.024;

    // -------------------------------------------------------------------------
    // ROS interfaces and TF listener
    // -------------------------------------------------------------------------
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Publisher<visual_servoing::msg::DetectedGoalArray>::SharedPtr detected_goals_pub_;

    rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedPtr save_pose_client_;

    // -------------------------------------------------------------------------
    // AprilTag detector state
    // -------------------------------------------------------------------------
    apriltag_detector_t *td_ = nullptr;
    apriltag_family_t *tf_ = nullptr;

    // -------------------------------------------------------------------------
    // Camera calibration parameters received from CameraInfo
    // -------------------------------------------------------------------------
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;

    // -------------------------------------------------------------------------
    // OpenCV display and user interaction state
    // -------------------------------------------------------------------------
    std::unique_ptr<VisualServoingDisplay> display_;

    cv::Rect save_button_rect_{10, 10, 180, 40};
    std::string save_status_ = "Ready";

    // -------------------------------------------------------------------------
    // Visible tag cache used by the SAVE POSE button
    // -------------------------------------------------------------------------
    std::vector<int> visible_tag_ids_;
    std::mutex visible_tags_mutex_;

    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    // Create the AprilTag detector and register the tag family used by the
    // system. The current setup uses tag36h11 tags.
    void initializeAprilTagDetector()
    {
        td_ = apriltag_detector_create();
        tf_ = tag36h11_create();

        if (!td_ || !tf_)
        {
            throw std::runtime_error("Failed to initialize AprilTag detector");
        }

        apriltag_detector_add_family(td_, tf_);
    }

    // Create the TF buffer and listener used to transform tag poses from the
    // camera frame to the robot target frame.
    void initializeTf()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // Declare and read ROS parameters.
    void loadParameters()
    {
        this->declare_parameter<std::string>("camera_type", "oak");
        this->declare_parameter<bool>("fullscreen_display", true);
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<double>("tag_size", 0.024);

        camera_type_ = this->get_parameter("camera_type").as_string();
        fullscreen_display_ = this->get_parameter("fullscreen_display").as_bool();
        target_frame_ = this->get_parameter("target_frame").as_string();
        tag_size_ = this->get_parameter("tag_size").as_double();
    }

    // Select camera topics from a simple camera_type parameter.
    // This keeps launch commands short while supporting multiple cameras.
    void configureCameraTopics()
    {
        if (camera_type_ == "oak")
        {
            image_topic_ = "/oak/rgb/image_raw";
            camera_info_topic_ = "/oak/rgb/camera_info";
            optical_frame_ = "oak_rgb_camera_optical_frame";
            window_name_ = "Visual Servoing OAK";
            return;
        }

        if (camera_type_ == "realsense")
        {
            image_topic_ = "/camera/camera/color/image_raw";
            camera_info_topic_ = "/camera/camera/color/camera_info";
            optical_frame_ = "camera_color_optical_frame";
            window_name_ = "Visual Servoing Realsense";
            return;
        }

        RCLCPP_FATAL(
            this->get_logger(),
            "Unknown camera_type: '%s'. Use 'oak' or 'realsense'.",
            camera_type_.c_str()
        );

        throw std::runtime_error("Invalid camera_type");
    }

    // Print the final configuration at startup.
    void logConfiguration()
    {
        RCLCPP_INFO(this->get_logger(), "Using camera_type: %s", camera_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera info topic: %s", camera_info_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Optical frame: %s", optical_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Tag size: %.3f m", tag_size_);
    }

    // Create all ROS publishers, subscribers, and service clients used by the node.
    void initializeRosInterfaces()
    {
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
    }

    // Initialize the OpenCV display helper and connect the mouse callback used
    // by the SAVE POSE button.
    void initializeDisplay()
    {
        display_ = std::make_unique<VisualServoingDisplay>(
            window_name_,
            fullscreen_display_
        );

        display_->setMouseCallback(&VisualServoingNode::mouseCallback, this);
    }

    // -------------------------------------------------------------------------
    // Mouse / save button
    // -------------------------------------------------------------------------

    // Static wrapper required by OpenCV. It forwards mouse events to the node.
    static void mouseCallback(int event, int x, int y, int flags, void *userdata)
    {
        (void)flags;

        if (!userdata)
        {
            return;
        }

        auto *self = static_cast<VisualServoingNode *>(userdata);
        self->handleMouse(event, x, y);
    }

    // Handle mouse clicks inside the OpenCV window.
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

    // Trigger pose saving for the first currently visible tag.
    // The actual save operation is delegated to /visual_servoing/save_current_tag_goal.
    void triggerSave()
    {
        if (!save_pose_client_ || !save_pose_client_->service_is_ready())
        {
            save_status_ = "Service not ready";

            RCLCPP_WARN(
                this->get_logger(),
                "Service /visual_servoing/save_current_tag_goal not ready"
            );

            return;
        }

        const int tag_id = getFirstVisibleTagId();

        if (tag_id < 0)
        {
            save_status_ = "No visible tag";
            RCLCPP_WARN(this->get_logger(), "No visible tag to save");
            return;
        }

        auto request =
            std::make_shared<visual_servoing::srv::SaveCurrentTagGoal::Request>();

        request->tag_id = tag_id;
        request->label = "pose";

        save_status_ = "Saving tag " + std::to_string(tag_id);

        save_pose_client_->async_send_request(
            request,
            [this, tag_id](
                rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedFuture future
            )
            {
                handleSaveResponse(tag_id, future);
            }
        );
    }

    // Return the first detected tag currently visible in the image.
    // Returns -1 when no tag is visible.
    int getFirstVisibleTagId()
    {
        std::lock_guard<std::mutex> lock(visible_tags_mutex_);

        if (visible_tag_ids_.empty())
        {
            return -1;
        }

        return visible_tag_ids_.front();
    }

    // Handle the asynchronous response from the save pose service.
    void handleSaveResponse(
        int tag_id,
        rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedFuture future
    )
    {
        try
        {
            const auto response = future.get();

            if (response->success)
            {
                save_status_ = "Saved tag " + std::to_string(tag_id);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Saved pose for tag %d",
                    tag_id
                );

                return;
            }

            save_status_ = "Save failed";

            RCLCPP_WARN(
                this->get_logger(),
                "Save failed for tag %d: %s",
                tag_id,
                response->message.c_str()
            );
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

    // -------------------------------------------------------------------------
    // Camera callbacks
    // -------------------------------------------------------------------------

    // Store the intrinsic camera parameters required by AprilTag pose estimation.
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
    }

    // Main image processing pipeline:
    //   1. Check camera intrinsics.
    //   2. Convert ROS image to OpenCV.
    //   3. Detect AprilTags.
    //   4. Estimate each tag pose.
    //   5. Transform pose to the target frame when possible.
    //   6. Publish detected goals.
    //   7. Draw visualization overlays.
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!hasCameraIntrinsics())
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

        if (!convertImage(msg, color))
        {
            return;
        }

        zarray_t *detections = detectTags(color);

        if (!detections)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "AprilTag detection failed"
            );

            return;
        }

        resetVisibleTags();

        auto detected_goals_msg = createDetectedGoalArrayMessage();

        processDetections(
            detections,
            msg->header.stamp,
            msg->header.frame_id,
            color,
            detected_goals_msg
        );

        apriltag_detections_destroy(detections);

        detected_goals_pub_->publish(detected_goals_msg);

        display_->drawSaveButton(color, save_button_rect_, save_status_);
        display_->show(color);
    }

    // Check whether CameraInfo has already been received.
    bool hasCameraIntrinsics() const
    {
        return fx_ != 0.0 && fy_ != 0.0;
    }

    // Convert the ROS image message into a modifiable OpenCV BGR image.
    bool convertImage(
        const sensor_msgs::msg::Image::SharedPtr &msg,
        cv::Mat &color
    )
    {
        try
        {
            color = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            return true;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "cv_bridge exception: %s",
                e.what()
            );

            return false;
        }
    }

    // Convert the BGR image to grayscale and run the AprilTag detector.
    // The returned zarray_t must be destroyed with apriltag_detections_destroy().
    zarray_t *detectTags(const cv::Mat &color)
    {
        cv::Mat gray;
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

        image_u8_t image_header = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        return apriltag_detector_detect(td_, &image_header);
    }

    // Create the outgoing message container.
    // By default, the header frame is the target frame. It can be updated later
    // if fallback camera-frame poses are published.
    visual_servoing::msg::DetectedGoalArray createDetectedGoalArrayMessage()
    {
        visual_servoing::msg::DetectedGoalArray message;
        message.header.stamp = this->now();
        message.header.frame_id = target_frame_;

        return message;
    }

    // Iterate over all detected AprilTags and process them one by one.
    void processDetections(
        zarray_t *detections,
        const rclcpp::Time &image_stamp,
        const std::string &image_frame_id,
        cv::Mat &color,
        visual_servoing::msg::DetectedGoalArray &detected_goals_msg
    )
    {
        const int number_of_detections = zarray_size(detections);

        for (int i = 0; i < number_of_detections; ++i)
        {
            apriltag_detection_t *detection = nullptr;
            zarray_get(detections, i, &detection);

            if (!detection)
            {
                continue;
            }

            processSingleDetection(
                detection,
                image_stamp,
                image_frame_id,
                color,
                detected_goals_msg
            );
        }
    }

    // Process one detected AprilTag:
    //   - estimate its pose in the camera frame,
    //   - draw it in the image,
    //   - transform it to the target frame if TF is available,
    //   - otherwise publish a fallback pose in the camera frame.
    void processSingleDetection(
        apriltag_detection_t *detection,
        const rclcpp::Time &image_stamp,
        const std::string &image_frame_id,
        cv::Mat &color,
        visual_servoing::msg::DetectedGoalArray &detected_goals_msg
    )
    {
        const geometry_msgs::msg::Pose tag_pose_camera =
            estimateTagPose(detection);

        geometry_msgs::msg::PoseStamped tag_pose_camera_stamped;
        tag_pose_camera_stamped.header.stamp = image_stamp;
        tag_pose_camera_stamped.header.frame_id = image_frame_id;
        tag_pose_camera_stamped.pose = tag_pose_camera;

        display_->drawDetectedTag(
            color,
            detection->id,
            detection->p,
            detection->c
        );

        geometry_msgs::msg::PoseStamped tag_pose_target;

        if (transformPoseToTarget(tag_pose_camera_stamped, tag_pose_target))
        {
            addDetectedGoal(
                detected_goals_msg,
                detection->id,
                tag_pose_target.pose,
                target_frame_
            );

            addVisibleTag(detection->id);
            return;
        }

        addDetectedGoal(
            detected_goals_msg,
            detection->id,
            tag_pose_camera,
            image_frame_id
        );

        addVisibleTag(detection->id);
    }

    // Estimate the 3D pose of one AprilTag using camera intrinsics and tag size.
    geometry_msgs::msg::Pose estimateTagPose(apriltag_detection_t *detection)
    {
        apriltag_detection_info_t info;
        info.det = detection;
        info.tagsize = tag_size_;
        info.fx = fx_;
        info.fy = fy_;
        info.cx = cx_;
        info.cy = cy_;

        apriltag_pose_t pose;
        const double error = estimate_tag_pose(&info, &pose);
        (void)error;

        return convertAprilTagPoseToRosPose(pose);
    }

    // Convert the AprilTag C pose representation into a ROS geometry_msgs/Pose.
    geometry_msgs::msg::Pose convertAprilTagPoseToRosPose(
        const apriltag_pose_t &pose
    ) const
    {
        const double x = matd_get(pose.t, 0, 0);
        const double y = matd_get(pose.t, 1, 0);
        const double z = matd_get(pose.t, 2, 0);

        Eigen::Matrix3d rotation;
        rotation <<
            matd_get(pose.R, 0, 0), matd_get(pose.R, 0, 1), matd_get(pose.R, 0, 2),
            matd_get(pose.R, 1, 0), matd_get(pose.R, 1, 1), matd_get(pose.R, 1, 2),
            matd_get(pose.R, 2, 0), matd_get(pose.R, 2, 1), matd_get(pose.R, 2, 2);

        Eigen::Quaterniond quaternion(rotation);
        quaternion.normalize();

        geometry_msgs::msg::Pose ros_pose;
        ros_pose.position.x = x;
        ros_pose.position.y = y;
        ros_pose.position.z = z;
        ros_pose.orientation.x = quaternion.x();
        ros_pose.orientation.y = quaternion.y();
        ros_pose.orientation.z = quaternion.z();
        ros_pose.orientation.w = quaternion.w();

        return ros_pose;
    }

    // Try to transform a pose from the camera frame to the configured target frame.
    // Returns false when the TF tree is not available yet.
    bool transformPoseToTarget(
        const geometry_msgs::msg::PoseStamped &input_pose,
        geometry_msgs::msg::PoseStamped &output_pose
    )
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
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Could not transform from %s to %s: %s. Displaying tag in camera image only.",
                input_pose.header.frame_id.c_str(),
                target_frame_.c_str(),
                ex.what()
            );

            return false;
        }
    }

    // Add one detected tag pose to the outgoing DetectedGoalArray message.
    void addDetectedGoal(
        visual_servoing::msg::DetectedGoalArray &detected_goals_msg,
        int tag_id,
        const geometry_msgs::msg::Pose &pose,
        const std::string &frame_id
    )
    {
        visual_servoing::msg::DetectedGoal detected_goal;
        detected_goal.id = tag_id;
        detected_goal.pose = pose;

        // NOTE:
        // DetectedGoalArray currently stores a single frame_id in the message
        // header. For now, all published poses are expected to share the same
        // frame. If mixed frames are needed later, DetectedGoal.msg should
        // include its own frame_id per goal.
        detected_goals_msg.header.frame_id = frame_id;
        detected_goals_msg.goals.push_back(detected_goal);
    }

    // Clear the list of tags visible in the current frame.
    void resetVisibleTags()
    {
        std::lock_guard<std::mutex> lock(visible_tags_mutex_);
        visible_tag_ids_.clear();
    }

    // Register one tag as visible in the current frame.
    void addVisibleTag(int tag_id)
    {
        std::lock_guard<std::mutex> lock(visible_tags_mutex_);
        visible_tag_ids_.push_back(tag_id);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<VisualServoingNode>());

    rclcpp::shutdown();

    return 0;
}