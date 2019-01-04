#ifndef TOOL_FRAME_DRAWER_HPP
#define TOOL_FRAME_DRAWER_HPP

#include <mutex>
#include <string>

#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>

#include <common/messages/visualizer_log_t.hpp>

namespace maav
{
namespace tools
{
class FrameDrawer
{
public:
    FrameDrawer(YAML::Node config);

    void update(const visualizer_log_t& msg);
    cv::Mat getDrawnFrame();

private:
    void drawTextInfo(const visualizer_log_t& msg);
    void copyImages(const visualizer_log_t& msg);

    double scale_;

    cv::Mat drawn_frame;
    cv::Mat rgb_im;
    cv::Mat depth_im;

    std::mutex mutex_;

    int num_tracked_map;
    int num_tracked_vo;
};
}
}

#endif
