/**
 * @file visual_servoing_node.cpp
 * @brief Main ROS 2 node for AprilTag-based visual servoing experiments.
 *
 * This node coordinates the full vision pipeline:
 * - receives camera images and camera calibration,
 * - detects AprilTags and estimates their 3D poses,
 * - transforms tag poses into a target robot frame when TF is available,
 * - publishes raw camera-frame tag poses for calibration,
 * - publishes target-frame detected goals when TF is available,
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
#include <stdexcept>
#include <string>
#include <vector>

#include "visual_servoing/apriltag_pose_estimator.hpp"
#include "visual_servoing/camera_config.hpp"
#include "visual_servoing/visual_servoing_display.hpp"
#include "visual_servoing/pose_transformer.hpp"
#include "visual_servoing/detected_goal_builder.hpp"
#include "visual_servoing/pose_save_client.hpp"

#include "visual_servoing/msg/detected_goal_array.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


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
        initializePoseSaveClient();
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

    /// Publisher for raw AprilTag pose in camera frame.
    rclcpp::Publisher<visual_servoing::msg::DetectedGoalArray>::SharedPtr tag_pose_camera_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tag_tf_broadcaster_;

    // -------------------------------------------------------------------------
    // Processing helpers
    // -------------------------------------------------------------------------

    /// Handles TF pose transformation into the configured target frame.
    std::unique_ptr<PoseTransformer> pose_transformer_;

    /// Handles AprilTag detection and camera-frame pose estimation.
    std::unique_ptr<AprilTagPoseEstimator> tag_pose_estimator_;

    /// Handles OpenCV drawing and window management.
    std::unique_ptr<VisualServoingDisplay> display_;

    /// Handles visible-tag tracking and SAVE POSE service calls.
    std::unique_ptr<PoseSaveClient> pose_save_client_;

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
        tag_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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
    * @brief Creates the helper responsible for SAVE POSE service calls.
    */
    void initializePoseSaveClient()
    {
        pose_save_client_ = std::make_unique<PoseSaveClient>(
            this,
            "/visual_servoing/save_current_tag_goal"
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
        tag_pose_camera_pub_ =
            this->create_publisher<visual_servoing::msg::DetectedGoalArray>(
                "/visual_servoing/tag_pose_camera",
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

        if (display_->isSaveButtonClicked(x, y))
        {
            pose_save_client_->triggerSave();
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

        pose_save_client_->resetVisibleTags();

        const rclcpp::Time image_time(msg->header.stamp);

        auto detected_goals_msg = DetectedGoalBuilder::createMessage(
            image_time,
            target_frame_
        );

        auto tag_pose_camera_msg = DetectedGoalBuilder::createMessage(
            image_time,
            msg->header.frame_id
        );

        processDetections(
            detected_tags,
            msg->header.stamp,
            msg->header.frame_id,
            color,
            detected_goals_msg,
            tag_pose_camera_msg
        );

        detected_goals_pub_->publish(detected_goals_msg);
        tag_pose_camera_pub_->publish(tag_pose_camera_msg);

        display_->drawSaveButton(
            color,
            pose_save_client_->status()
        );
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
     * @brief Processes all detected tags from one image frame.
     */
    void processDetections(
        const std::vector<DetectedTagPose> &detected_tags,
        const rclcpp::Time &image_stamp,
        const std::string &image_frame_id,
        cv::Mat &color,
        visual_servoing::msg::DetectedGoalArray &detected_goals_msg,
        visual_servoing::msg::DetectedGoalArray &tag_pose_camera_msg
    )
    {
        for (const auto &detected_tag : detected_tags)
        {
            processSingleDetection(
                detected_tag,
                image_stamp,
                image_frame_id,
                color,
                detected_goals_msg,
                tag_pose_camera_msg
            );
        }
    }

    /**
    * @brief Processes one detected AprilTag.
    *
    * The raw camera-frame pose is always published on /visual_servoing/tag_pose_camera.
    * If TF is available, the pose is also published in target_frame_.
    */
    void processSingleDetection(
        const DetectedTagPose &detected_tag,
        const rclcpp::Time &image_stamp,
        const std::string &image_frame_id,
        cv::Mat &color,
        visual_servoing::msg::DetectedGoalArray &detected_goals_msg,
        visual_servoing::msg::DetectedGoalArray &tag_pose_camera_msg
    )
    {
        geometry_msgs::msg::PoseStamped tag_pose_camera_stamped;
        tag_pose_camera_stamped.header.stamp = image_stamp;
        tag_pose_camera_stamped.header.frame_id = image_frame_id;
        tag_pose_camera_stamped.pose = detected_tag.pose_camera;


        geometry_msgs::msg::TransformStamped tag_tf;

        tag_tf.header.stamp = image_stamp;
        tag_tf.header.frame_id = image_frame_id;
        tag_tf.child_frame_id = "tag_" + std::to_string(detected_tag.id);

        tag_tf.transform.translation.x = detected_tag.pose_camera.position.x;
        tag_tf.transform.translation.y = detected_tag.pose_camera.position.y;
        tag_tf.transform.translation.z = detected_tag.pose_camera.position.z;
        tag_tf.transform.rotation = detected_tag.pose_camera.orientation;

        tag_tf_broadcaster_->sendTransform(tag_tf);

        // Store raw camera-frame tag pose for hand-eye calibration.
        // This is C_T_T: tag pose expressed in the camera frame.
        DetectedGoalBuilder::addGoal(
            tag_pose_camera_msg,
            detected_tag.id,
            detected_tag.pose_camera,
            image_frame_id
        );

        display_->drawDetectedTag(
            color,
            detected_tag.id,
            detected_tag.corners,
            detected_tag.center
        );

        geometry_msgs::msg::PoseStamped tag_pose_target;

        if (pose_transformer_->transformToTargetFrame(tag_pose_camera_stamped, tag_pose_target))
        {
            DetectedGoalBuilder::addGoal(
                detected_goals_msg,
                detected_tag.id,
                tag_pose_target.pose,
                target_frame_
            );

            pose_save_client_->addVisibleTag(detected_tag.id);
        }
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
