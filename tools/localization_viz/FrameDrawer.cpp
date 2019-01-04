#include <iostream>
#include <limits>
#include <sstream>

#include <gnc/slam/Tracking.h>
#include <gnc/slam/VisualizerLink.hpp>
#include "FrameDrawer.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using maav::gnc::slam::Tracking;
using maav::gnc::slam::KP_MAP_BIT;
using maav::gnc::slam::KP_VO_BIT;

namespace maav
{
namespace tools
{
FrameDrawer::FrameDrawer(YAML::Node config)
    : scale_(5000),
      drawn_frame(cv::Mat(480 * 2, 640, CV_8UC3, cv::Scalar(0, 0, 0))),
      rgb_im(cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0))),
      depth_im(cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0))),
      num_tracked_map(0),
      num_tracked_vo(0)
{
}

void FrameDrawer::update(const visualizer_log_t& msg)
{
    std::unique_lock<std::mutex> lock(mutex_);
    copyImages(msg);

    if (msg.state == Tracking::OK)  // TRACKING
    {
        num_tracked_map = 0;
        num_tracked_vo = 0;
        const float r = 3;
        const int n = msg.num_keypoints;
        for (int i = 0; i < n; i++)
        {
            if (msg.keypoints[i].info & KP_MAP_BIT || msg.keypoints[i].info & KP_VO_BIT)
            {
                cv::Point2f pt, pt_r;
                pt.x = msg.keypoints[i].x;
                pt.y = msg.keypoints[i].y;

                pt_r.x = r;
                pt_r.y = r;

                // This is a match to a MapPoint in the map
                if (msg.keypoints[i].info & KP_MAP_BIT)
                {
                    cv::rectangle(rgb_im, pt + pt_r, pt - pt_r, cv::Scalar(0, 255, 0));
                    cv::circle(rgb_im, pt, 2, cv::Scalar(0, 255, 0), -1);
                    num_tracked_map++;
                }
                else  // This is match to a "visual odometry" MapPoint created
                      // in the last frame
                {
                    cv::rectangle(rgb_im, pt + pt_r, pt - pt_r, cv::Scalar(255, 0, 0));
                    cv::circle(rgb_im, pt, 2, cv::Scalar(255, 0, 0), -1);
                    num_tracked_vo++;
                }
            }
        }
    }

    drawTextInfo(msg);
}

cv::Mat FrameDrawer::getDrawnFrame()
{
    std::unique_lock<std::mutex> lock(mutex_);
    return drawn_frame;
}

void FrameDrawer::drawTextInfo(const visualizer_log_t& msg)
{
    cv::Mat stacked;
    cv::vconcat(rgb_im, depth_im, stacked);

    std::stringstream s;
    if (msg.state == Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if (msg.state == Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if (msg.state == Tracking::OK)
    {
        if (!msg.only_tracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = msg.num_keyframes;
        int nMPs = msg.num_points;

        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << num_tracked_map;
        if (num_tracked_vo > 0) s << ", + VO matches: " << num_tracked_vo;
    }
    else if (msg.state == Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if (msg.state == Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline = 0;
    cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

    drawn_frame = cv::Mat(stacked.rows + textSize.height + 10, stacked.cols, stacked.type());
    stacked.copyTo(drawn_frame.rowRange(0, stacked.rows).colRange(0, stacked.cols));
    drawn_frame.rowRange(stacked.rows, drawn_frame.rows) =
        cv::Mat::zeros(textSize.height + 10, stacked.cols, stacked.type());
    cv::putText(drawn_frame, s.str(), cv::Point(5, drawn_frame.rows - 5), cv::FONT_HERSHEY_PLAIN, 1,
        cv::Scalar(255, 255, 255), 1, 8);
}

void FrameDrawer::copyImages(const visualizer_log_t& msg)
{
    rgb_im = cv::Mat(cv::Size(640, 480), CV_8UC3, (uint8_t*)msg.img.rgb_image.raw_image.data(),
        cv::Mat::AUTO_STEP);

    drawn_frame = rgb_im;

    depth_im = cv::Mat(cv::Size(640, 480), CV_16UC1,
        (uint16_t*)msg.img.depth_image.raw_image.data(), cv::Mat::AUTO_STEP);

    constexpr double MAX_DEPTH_SCALED = 10 * 1000;
    cv::convertScaleAbs(depth_im, depth_im, 255.0 / MAX_DEPTH_SCALED, 0);

    applyColorMap(depth_im, depth_im, cv::COLORMAP_HSV);
}
}
}
