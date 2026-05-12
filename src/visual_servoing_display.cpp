#include "visual_servoing/visual_servoing_display.hpp"

VisualServoingDisplay::VisualServoingDisplay(
    const std::string &window_name,
    bool fullscreen_display
)
    : window_name_(window_name)
{
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);

    if (fullscreen_display)
    {
        cv::setWindowProperty(
            window_name_,
            cv::WND_PROP_FULLSCREEN,
            cv::WINDOW_FULLSCREEN
        );
    }
}

VisualServoingDisplay::~VisualServoingDisplay()
{
    cv::destroyWindow(window_name_);
}

void VisualServoingDisplay::setMouseCallback(
    void (*callback)(int event, int x, int y, int flags, void *userdata),
    void *userdata
)
{
    cv::setMouseCallback(window_name_, callback, userdata);
}

void VisualServoingDisplay::drawDetectedTag(
    cv::Mat &image,
    int tag_id,
    const double corners[4][2],
    const double center[2]
)
{
    for (int j = 0; j < 4; j++)
    {
        cv::line(
            image,
            cv::Point(corners[j][0], corners[j][1]),
            cv::Point(corners[(j + 1) % 4][0], corners[(j + 1) % 4][1]),
            cv::Scalar(0, 0, 255),
            2
        );
    }

    cv::putText(
        image,
        std::to_string(tag_id),
        cv::Point(static_cast<int>(center[0]), static_cast<int>(center[1])),
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        cv::Scalar(0, 255, 0),
        2
    );
}

void VisualServoingDisplay::drawSaveButton(
    cv::Mat &image,
    const cv::Rect &button_rect,
    const std::string &save_status
)
{
    cv::rectangle(
        image,
        button_rect,
        cv::Scalar(255, 0, 0),
        cv::FILLED
    );

    cv::putText(
        image,
        "SAVE POSE",
        cv::Point(button_rect.x + 15, button_rect.y + 27),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(255, 255, 255),
        2
    );

    cv::putText(
        image,
        save_status,
        cv::Point(10, 65),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 255),
        2
    );
}

void VisualServoingDisplay::show(cv::Mat &image)
{
    cv::imshow(window_name_, image);
    cv::waitKey(1);
}
