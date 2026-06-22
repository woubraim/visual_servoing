/**
 * @file handeye_tf_publisher.cpp
 * @brief Publish calibrated end-effector to camera transform as static TF.
 *
 * This version reads the transform directly from a YAML file:
 *
 *   tx ty tz qx qy qz qw
 *
 * and publishes:
 *
 *   parent frame: ee_frame
 *   child frame:  camera_frame
 *
 * In our setup:
 *
 *   ee_frame     = camera_explorer
 *   camera_frame = oak_parent_frame
 */

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/core.hpp>


struct HandEyeCalibration
{
    std::string ee_frame;
    std::string camera_frame;

    double tx;
    double ty;
    double tz;

    double qx;
    double qy;
    double qz;
    double qw;
};


class HandEyeTfPublisher : public rclcpp::Node
{
public:
    HandEyeTfPublisher()
        : Node("handeye_tf_publisher")
    {
        declare_parameter<std::string>("yaml_path", "/home/woubraim/extender_ws/src/visual_servoing/yaml/handeye_tf.yaml");

        const std::string yaml_path =
            get_parameter("yaml_path").as_string();

        const HandEyeCalibration calib =
            readCalibrationYaml(yaml_path);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now();
        transform.header.frame_id = calib.ee_frame;
        transform.child_frame_id = calib.camera_frame;

        transform.transform.translation.x = calib.tx;
        transform.transform.translation.y = calib.ty;
        transform.transform.translation.z = calib.tz;

        tf2::Quaternion q(calib.qx, calib.qy, calib.qz, calib.qw);

        if (q.length2() < 1e-12)
        {
            throw std::runtime_error("Invalid quaternion in YAML: norm is zero");
        }

        q.normalize();

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        broadcaster_->sendTransform(transform);

        const double translation_norm = std::sqrt(
            calib.tx * calib.tx +
            calib.ty * calib.ty +
            calib.tz * calib.tz
        );

        RCLCPP_INFO(
            get_logger(),
            "Loaded calibration YAML: %s",
            yaml_path.c_str()
        );

        RCLCPP_INFO(
            get_logger(),
            "Published static TF: %s -> %s",
            calib.ee_frame.c_str(),
            calib.camera_frame.c_str()
        );

        RCLCPP_INFO(
            get_logger(),
            "translation: [%.6f, %.6f, %.6f] m, norm = %.6f m",
            calib.tx,
            calib.ty,
            calib.tz,
            translation_norm
        );

        RCLCPP_INFO(
            get_logger(),
            "rotation quaternion xyzw: [%.6f, %.6f, %.6f, %.6f]",
            q.x(),
            q.y(),
            q.z(),
            q.w()
        );
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

    HandEyeCalibration readCalibrationYaml(const std::string &yaml_path) const
    {
        cv::FileStorage fs(yaml_path, cv::FileStorage::READ);

        if (!fs.isOpened())
        {
            throw std::runtime_error("Could not open calibration YAML: " + yaml_path);
        }

        HandEyeCalibration calib;

        fs["ee_frame"] >> calib.ee_frame;
        fs["camera_frame"] >> calib.camera_frame;

        fs["tx"] >> calib.tx;
        fs["ty"] >> calib.ty;
        fs["tz"] >> calib.tz;

        fs["qx"] >> calib.qx;
        fs["qy"] >> calib.qy;
        fs["qz"] >> calib.qz;
        fs["qw"] >> calib.qw;

        fs.release();

        if (calib.ee_frame.empty())
        {
            throw std::runtime_error("Missing ee_frame in YAML");
        }

        if (calib.camera_frame.empty())
        {
            throw std::runtime_error("Missing camera_frame in YAML");
        }

        const double q_norm2 =
            calib.qx * calib.qx +
            calib.qy * calib.qy +
            calib.qz * calib.qz +
            calib.qw * calib.qw;

        if (q_norm2 < 1e-12)
        {
            throw std::runtime_error("Invalid quaternion in YAML");
        }

        return calib;
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<HandEyeTfPublisher>());

    rclcpp::shutdown();

    return 0;
}