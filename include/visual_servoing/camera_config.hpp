#pragma once

#include <stdexcept>
#include <string>


struct CameraConfig
{
    std::string camera_type;
    std::string image_topic;
    std::string camera_info_topic;
    std::string optical_frame;
    std::string window_name;

    static CameraConfig fromCameraType(const std::string &camera_type);
};
