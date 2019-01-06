#ifndef VISUALIZER_LINK_HPP
#define VISUALIZER_LINK_HPP

#include <mutex>
#include <string>

#include <zcm/zcm-cpp.hpp>

#include <gnc/slam/Map.h>
#include <gnc/slam/Tracking.h>
#include <common/messages/visualizer_log_t.hpp>

namespace maav
{
namespace gnc
{
namespace slam
{
class Tracking;

static constexpr int8_t PT_REF_BIT = 0b01, PT_MAP_BIT = 0b10;
static constexpr int8_t KP_VO_BIT = 0b01, KP_MAP_BIT = 0b10;

class VisualizerLink
{
public:
    VisualizerLink(const std::string& zcm_url, Map* map, bool send_images, bool send_points,
        bool send_keyframes);

    void updateTracking(Tracking* tracker);

    void run();

    void setCurrentPose(const cv::Mat& pose);

private:
    void convertRgbImage(const cv::Mat& im);
    void convertDepthImage(const cv::Mat& im);

    zcm::ZCM zcm;

    Map* map_;

    // Tracking
    std::mutex tracking_mutex_;
    visualizer_log_t last_frame;

    cv::Mat current_pose_;

    bool send_images_;
    bool send_points_;
    bool send_keyframes_;
};
}
}
}

#endif
