#pragma once

#include <opencv2/opencv.hpp>

#include <array>
#include <string>


class VisualServoingDisplay
{
public:
    VisualServoingDisplay(
        const std::string &window_name,
        bool fullscreen_display
    );

    ~VisualServoingDisplay();

    void setMouseCallback(
        void (*callback)(int event, int x, int y, int flags, void *userdata),
        void *userdata
    );

    void drawDetectedTag(
        cv::Mat &image,
        int tag_id,
        const std::array<cv::Point2d, 4> &corners,
        const cv::Point2d &center
    );

    void drawSaveButton(
        cv::Mat &image,
        const cv::Rect &button_rect,
        const std::string &save_status
    );

    void show(cv::Mat &image);

private:
    std::string window_name_;
};