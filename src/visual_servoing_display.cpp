/**
 * @file visual_servoing_display.cpp
 * @brief Handles all OpenCV visualization for the visual servoing node.
 *
 * This class is responsible only for display-related operations:
 * - creating and destroying the OpenCV window,
 * - drawing detected AprilTag borders and IDs,
 * - drawing the SAVE POSE button and its status text,
 * - displaying the final image.
 *
 * It does not contain ROS logic, AprilTag logic, or TF logic.
 */

#include "visual_servoing/visual_servoing_display.hpp"

#include <array>
#include <string>

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

/**
 * @brief Registers a mouse callback on the OpenCV window.
 *
 * OpenCV requires a C-style callback, so the node passes a static wrapper and
 * its own pointer through userdata.
 */
void VisualServoingDisplay::setMouseCallback(
    void (*callback)(int event, int x, int y, int flags, void *userdata),
    void *userdata
)
{
    cv::setMouseCallback(window_name_, callback, userdata);
}


/**
 * @brief Draws the border and ID of one detected AprilTag.
 */
void VisualServoingDisplay::drawDetectedTag(
    cv::Mat &image,
    int tag_id,
    const std::array<cv::Point2d, 4> &corners,
    const cv::Point2d &center
)
{
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point p1(
            static_cast<int>(corners[i].x),
            static_cast<int>(corners[i].y)
        );

        const cv::Point p2(
            static_cast<int>(corners[(i + 1) % 4].x),
            static_cast<int>(corners[(i + 1) % 4].y)
        );

        cv::line(
            image,
            p1,
            p2,
            cv::Scalar(0, 0, 255),
            2
        );
    }

    cv::putText(
        image,
        std::to_string(tag_id),
        cv::Point(
            static_cast<int>(center.x),
            static_cast<int>(center.y)
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        cv::Scalar(0, 255, 0),
        2
    );
}

/**
 * @brief Draws the SAVE POSE button and the current save status.
 */
void VisualServoingDisplay::drawSaveButton(
    cv::Mat &image,
    const std::string &save_status
)
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
        save_status,
        cv::Point(10, 65),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 255),
        2
    );
}

/**
 * @brief Displays the current frame and lets OpenCV process UI events.
 */
void VisualServoingDisplay::show(cv::Mat &image)
{
    cv::imshow(window_name_, image);
    cv::waitKey(1);
}
/**
 * @brief Checks if the SAVE POSE button was clicked.
 */
bool VisualServoingDisplay::isSaveButtonClicked(int x, int y) const
{
    return save_button_rect_.contains(cv::Point(x, y));
}
