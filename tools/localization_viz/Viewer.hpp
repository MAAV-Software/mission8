#ifndef TOOL_VIEWER_HPP
#define TOOL_VIEWER_HPP

#include <chrono>
#include <mutex>
#include <string>

#include <pangolin/pangolin.h>
#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include "FrameDrawer.hpp"
#include "MapDrawer.hpp"

#include <common/messages/state_t.hpp>
#include <common/messages/visualizer_log_t.hpp>
#include <gnc/State.hpp>

namespace maav
{
namespace tools
{
/**
* @class Extracted data visualizer tool from ORBSLAM2
*/
class Viewer
{
public:
    Viewer(YAML::Node config, std::shared_ptr<FrameDrawer> frame_drawer,
        std::shared_ptr<MapDrawer> map_drawer, zcm::ZCM& zcm_node);

    void createWindow();
    void drawWindow(pangolin::OpenGlRenderState& s_cam, pangolin::View& d_cam);
    void setLocalizationMode(bool on);
    void reset();

    // Main thread function. Draw points, keyframes, the current camera pose and
    // the last processed frame. Drawing is refreshed according to the camera
    // fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    void updateLocalizer(
        const zcm::ReceiveBuffer*, const std::string&, const visualizer_log_t* msg);
    void updateEstimator(const zcm::ReceiveBuffer*, const std::string&, const state_t* msg);

private:
    const std::string PANGOLIN_WINDOW_NAME = "ORB-SLAM2: Map Viewer";
    const std::string FORWARD_WINDOW_NAME = "Forward Camera";

    double fps_;
    int period_;

    // Viewport variables
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    pangolin::OpenGlMatrix Twc;

    bool follow_;
    bool localization_mode_;

    using Checkbox = pangolin::Var<bool>;
    using CheckboxPtr = std::unique_ptr<Checkbox>;
    CheckboxPtr menuFollowCamera;
    CheckboxPtr menuShowPoints;
    CheckboxPtr menuShowKeyFrames;
    CheckboxPtr menuShowGraph;
    CheckboxPtr menuLocalizationMode;
    CheckboxPtr menuKalmanState;
    CheckboxPtr menuKalmanCovariance;
    CheckboxPtr menuKalmanVelocity;
    CheckboxPtr menuReset;

    // Stopping utilities
    bool Stop();
    bool CheckFinish();
    void SetFinish();

    bool finish_requested_;
    bool finished_;
    std::mutex finish_mutex_;

    bool stop_requested_;
    bool stopped_;
    std::mutex stop_mutex_;

    std::shared_ptr<FrameDrawer> frame_drawer_;
    std::shared_ptr<MapDrawer> map_drawer_;

    std::mutex data_mutex_;
    visualizer_log_t data;
};
}
}

#endif
