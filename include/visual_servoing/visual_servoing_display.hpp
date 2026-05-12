#pragma once

#include <opencv2/opencv.hpp>

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
        const double corners[4][2],
        const double center[2]
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
