/**
 * @file visual_servoing_node.cpp
 * @brief Main ROS 2 node for AprilTag-based visual servoing experiments.
 *
 * This node coordinates the full vision pipeline:
 * - receives camera images and camera calibration,
 * - detects AprilTags and estimates their 3D poses,
 * - transforms tag poses into a target robot frame when TF is available,
 * - falls back to camera-frame poses when TF is missing,
 * - publishes detected goals,
 * - displays an annotated OpenCV camera view,
 * - allows saving the currently visible tag pose through a ROS service.
 *
 * The implementation intentionally keeps the node focused on ROS orchestration.
 * Camera selection, AprilTag pose estimation, and OpenCV drawing are delegated
 * to dedicated helper classes.
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "visual_servoing/apriltag_pose_estimator.hpp"
#include "visual_servoing/camera_config.hpp"
#include "visual_servoing/visual_servoing_display.hpp"
#include "visual_servoing/pose_transformer.hpp"

#include "visual_servoing/msg/detected_goal.hpp"
#include "visual_servoing/msg/detected_goal_array.hpp"
#include "visual_servoing/srv/save_current_tag_goal.hpp"


/**
 * @brief ROS 2 node coordinating camera input, AprilTag pose estimation, TF,
 * visualization, and pose saving.
 *
 * Responsibilities kept in this class:
 * - ROS publishers, subscribers, and service client,
 * - TF transform from camera frame to target frame,
 * - coordination between AprilTagPoseEstimator and VisualServoingDisplay,
 * - user interaction for saving the currently visible tag pose.
 *
 * Responsibilities moved outside this class:
 * - camera-specific topic selection: CameraConfig,
 * - AprilTag detection and pose estimation: AprilTagPoseEstimator,
 * - OpenCV drawing and window management: VisualServoingDisplay.
 */
class VisualServoingNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs and initializes the visual servoing node.
     *
     * Parameter loading is done before creating the AprilTag estimator because
     * the estimator needs the configured tag size.
     */
    VisualServoingNode()
    : Node("visual_servoing_node")
    {
        loadParameters();
        initializeAprilTagEstimator();
        initializeTf();
        initializePoseTransformer();
        configureCamera();
        logConfiguration();
        initializeRosInterfaces();
        initializeDisplay();
    }

private:
    // -------------------------------------------------------------------------
    // Runtime configuration
    // -------------------------------------------------------------------------

    /// Camera type selected through the camera_type ROS parameter.
    std::string camera_type_;

    /// Camera-specific topics, frame names, and display window name.
    CameraConfig camera_config_;

    /// Whether the OpenCV display window should be opened in fullscreen mode.
    bool fullscreen_display_ = true;

    /// Frame in which tag poses should be published when TF is available.
    std::string target_frame_ = "base_link";

    /// Physical side length of the AprilTag in meters.
    double tag_size_ = 0.024;

    // -------------------------------------------------------------------------
    // ROS interfaces and TF
    // -------------------------------------------------------------------------

    /// TF buffer used to query transforms between camera and robot frames.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /// TF listener keeping the TF buffer updated.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /// Camera image subscriber.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    /// Camera calibration subscriber.
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    /// Publisher for detected tag goals.
    rclcpp::Publisher<visual_servoing::msg::DetectedGoalArray>::SharedPtr detected_goals_pub_;

    /// Client used by the SAVE POSE button.
    rclcpp::Client<visual_servoing::srv::SaveCurrentTagGoal>::SharedPtr save_pose_client_;

    // -------------------------------------------------------------------------
    // Processing helpers
    // -------------------------------------------------------------------------

    /// Handles TF pose transformation into the configured target frame.
    std::unique_ptr<PoseTransformer> pose_transformer_;

    /// Handles AprilTag detection and camera-frame pose estimation.
    std::unique_ptr<AprilTagPoseEstimator> tag_pose_estimator_;

    /// Handles OpenCV drawing and window management.
    std::unique_ptr<VisualServoingDisplay> display_;

    // -------------------------------------------------------------------------
    // Camera calibration
    // -------------------------------------------------------------------------

    /// Focal length in pixels along x.
    double fx_ = 0.0;

    /// Focal length in pixels along y.
    double fy_ = 0.0;

    /// Principal point x coordinate.
    double cx_ = 0.0;

    /// Principal point y coordinate.
    double cy_ = 0.0;

    // -------------------------------------------------------------------------
    // SAVE POSE UI state
    // -------------------------------------------------------------------------

    /// Rectangle defining the clickable SAVE POSE button area in the image.
    cv::Rect save_button_rect_{10, 10, 180, 40};

    /// Status text displayed under the SAVE POSE button.
    std::string save_status_ = "Ready";

    /// IDs of tags detected in the latest processed frame.
    std::vector<int> visible_tag_ids_;

    /// Protects visible_tag_ids_, which can be read from the mouse callback.
    std::mutex visible_tags_mutex_;

    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    /**
     * @brief Declares and reads all ROS parameters.
     */
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

    /**
     * @brief Creates the AprilTag estimator using the configured tag size.
     */
    void initializeAprilTagEstimator()
    {
        tag_pose_estimator_ = std::make_unique<AprilTagPoseEstimator>(tag_size_);
    }

    /**
     * @brief Creates the TF buffer and listener.
     */
    void initializeTf()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    /**
    * @brief Creates the helper responsible for TF pose transformations.
    */
    void initializePoseTransformer()
    {
        pose_transformer_ = std::make_unique<PoseTransformer>(
            tf_buffer_,
            this->get_logger(),
            this->get_clock(),
            target_frame_
        );
    }

    /**
     * @brief Loads camera-specific topics and display configuration.
     *
     * CameraConfig centralizes the mapping between a camera type name and the
     * ROS topics used by that camera.
     */
    void configureCamera()
    {
        try
        {
            camera_config_ = CameraConfig::fromCameraType(camera_type_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "%s", e.what());
            throw;
        }
    }

    /**
     * @brief Prints the effective runtime configuration.
     */
    void logConfiguration()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Using camera_type: %s",
            camera_config_.camera_type.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Image topic: %s",
            camera_config_.image_topic.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Camera info topic: %s",
            camera_config_.camera_info_topic.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Optical frame: %s",
            camera_config_.optical_frame.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Target frame: %s",
            target_frame_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Tag size: %.3f m",
            tag_size_
        );
    }

    /**
     * @brief Creates all ROS publishers, subscribers, and service clients.
     */
    void initializeRosInterfaces()
    {
        detected_goals_pub_ =
            this->create_publisher<visual_servoing::msg::DetectedGoalArray>(
                "/visual_servoing/detected_goals",
                10
            );

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_config_.image_topic,
            10,
            std::bind(&VisualServoingNode::imageCallback, this, std::placeholders::_1)
        );

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_config_.camera_info_topic,
            10,
            std::bind(&VisualServoingNode::cameraInfoCallback, this, std::placeholders::_1)
        );

        save_pose_client_ =
            this->create_client<visual_servoing::srv::SaveCurrentTagGoal>(
                "/visual_servoing/save_current_tag_goal"
            );
    }

    /**
     * @brief Initializes the OpenCV display and connects the mouse callback.
     */
    void initializeDisplay()
    {
        display_ = std::make_unique<VisualServoingDisplay>(
            camera_config_.window_name,
            fullscreen_display_
        );

        display_->setMouseCallback(&VisualServoingNode::mouseCallback, this);
    }

    // -------------------------------------------------------------------------
    // Mouse interaction and save button
    // -------------------------------------------------------------------------

    /**
     * @brief Static OpenCV mouse callback wrapper.
     *
     * OpenCV requires a C-style callback. The node instance is passed through
     * userdata and used to forward the event to handleMouse().
     */
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

    /**
     * @brief Handles mouse clicks in the OpenCV display window.
     */
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

    /**
     * @brief Requests saving the first currently visible tag pose.
     *
     * The actual save implementation is delegated to the
     * /visual_servoing/save_current_tag_goal service.
     */
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

    /**
     * @brief Returns the first visible tag ID from the latest frame.
     *
     * @return Tag ID if a tag is visible, otherwise -1.
     */
    int getFirstVisibleTagId()
    {
        std::lock_guard<std::mutex> lock(visible_tags_mutex_);

        if (visible_tag_ids_.empty())
        {
            return -1;
        }

        return visible_tag_ids_.front();
    }

    /**
     * @brief Handles the asynchronous response from the save pose service.
     */
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
    // Camera callbacks and image processing
    // -------------------------------------------------------------------------

    /**
     * @brief Stores camera intrinsics received from the CameraInfo topic.
     *
     * AprilTag pose estimation requires fx, fy, cx, and cy from the camera
     * calibration matrix K.
     */
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
    }

    /**
     * @brief Main image callback.
     *
     * Processing steps:
     * 1. Check that CameraInfo has been received.
     * 2. Convert the ROS image to an OpenCV BGR image.
     * 3. Detect AprilTags and estimate their camera-frame poses.
     * 4. Transform poses to target_frame_ when TF is available.
     * 5. Publish detected goals.
     * 6. Draw the display overlays.
     */
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

        const auto detected_tags = tag_pose_estimator_->detect(
            color,
            fx_,
            fy_,
            cx_,
            cy_
        );

        resetVisibleTags();

        auto detected_goals_msg = createDetectedGoalArrayMessage();

        processDetections(
            detected_tags,
            msg->header.stamp,
            msg->header.frame_id,
            color,
            detected_goals_msg
        );

        detected_goals_pub_->publish(detected_goals_msg);

        display_->drawSaveButton(color, save_button_rect_, save_status_);
        display_->show(color);
    }

    /**
     * @brief Checks whether camera intrinsics have been received.
     */
    bool hasCameraIntrinsics() const
    {
        return fx_ != 0.0 && fy_ != 0.0;
    }

    /**
     * @brief Converts a ROS image message into a modifiable OpenCV BGR image.
     *
     * @return true if conversion succeeds, false otherwise.
     */
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

    /**
     * @brief Creates the outgoing detected-goals message.
     *
     * By default, the message frame is target_frame_. If TF is unavailable and
     * fallback camera-frame poses are published, the frame is updated later.
     */
    visual_servoing::msg::DetectedGoalArray createDetectedGoalArrayMessage()
    {
        visual_servoing::msg::DetectedGoalArray message;
        message.header.stamp = this->now();
        message.header.frame_id = target_frame_;

        return message;
    }

    /**
     * @brief Processes all detected tags from one image frame.
     */
    void processDetections(
        const std::vector<DetectedTagPose> &detected_tags,
        const rclcpp::Time &image_stamp,
        const std::string &image_frame_id,
        cv::Mat &color,
        visual_servoing::msg::DetectedGoalArray &detected_goals_msg
    )
    {
        for (const auto &detected_tag : detected_tags)
        {
            processSingleDetection(
                detected_tag,
                image_stamp,
                image_frame_id,
                color,
                detected_goals_msg
            );
        }
    }

    /**
     * @brief Processes one detected AprilTag.
     *
     * The tag is always drawn in the image. Its pose is published in target_frame_
     * when the TF transform is available. Otherwise, the camera-frame pose is
     * published so the vision pipeline remains testable without the robot TF tree.
     */
    void processSingleDetection(
        const DetectedTagPose &detected_tag,
        const rclcpp::Time &image_stamp,
        const std::string &image_frame_id,
        cv::Mat &color,
        visual_servoing::msg::DetectedGoalArray &detected_goals_msg
    )
    {
        geometry_msgs::msg::PoseStamped tag_pose_camera_stamped;
        tag_pose_camera_stamped.header.stamp = image_stamp;
        tag_pose_camera_stamped.header.frame_id = image_frame_id;
        tag_pose_camera_stamped.pose = detected_tag.pose_camera;

        display_->drawDetectedTag(
            color,
            detected_tag.id,
            detected_tag.corners,
            detected_tag.center
        );

        geometry_msgs::msg::PoseStamped tag_pose_target;

        if (pose_transformer_->transformToTargetFrame(tag_pose_camera_stamped, tag_pose_target))
        {
            addDetectedGoal(
                detected_goals_msg,
                detected_tag.id,
                tag_pose_target.pose,
                target_frame_
            );

            addVisibleTag(detected_tag.id);
            return;
        }

        addDetectedGoal(
            detected_goals_msg,
            detected_tag.id,
            detected_tag.pose_camera,
            image_frame_id
        );

        addVisibleTag(detected_tag.id);
    }

    /**
     * @brief Adds one detected tag pose to the outgoing message.
     *
     * @note DetectedGoalArray currently stores one shared header.frame_id.
     * Therefore, all poses in the message are expected to share the same frame.
     * If mixed frames are needed later, DetectedGoal.msg should include its own
     * frame_id per goal.
     */
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

        detected_goals_msg.header.frame_id = frame_id;
        detected_goals_msg.goals.push_back(detected_goal);
    }

    /**
     * @brief Clears the list of tags visible in the current frame.
     */
    void resetVisibleTags()
    {
        std::lock_guard<std::mutex> lock(visible_tags_mutex_);
        visible_tag_ids_.clear();
    }

    /**
     * @brief Registers one tag as visible in the current frame.
     */
    void addVisibleTag(int tag_id)
    {
        std::lock_guard<std::mutex> lock(visible_tags_mutex_);
        visible_tag_ids_.push_back(tag_id);
    }
};


/**
 * @brief Program entry point.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<VisualServoingNode>());

    rclcpp::shutdown();

    return 0;
}