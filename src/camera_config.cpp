#include "visual_servoing/camera_config.hpp"


CameraConfig CameraConfig::fromCameraType(const std::string &camera_type)
{
    if (camera_type == "oak")
    {
        return {
            "oak",
            "/oak/rgb/image_raw",
            "/oak/rgb/camera_info",
            "oak_rgb_camera_optical_frame",
            "Visual Servoing OAK"
        };
    }

    if (camera_type == "realsense")
    {
        return {
            "realsense",
            "/camera/camera/color/image_raw",
            "/camera/camera/color/camera_info",
            "camera_color_optical_frame",
            "Visual Servoing Realsense"
        };
    }

    throw std::runtime_error(
        "Unknown camera_type: '" + camera_type + "'. Use 'oak' or 'realsense'."
    );
}
