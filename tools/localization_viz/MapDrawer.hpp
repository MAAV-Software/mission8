#ifndef TOOL_MAP_DRAWER_HPP
#define TOOL_MAP_DRAWER_HPP

#include <pangolin/pangolin.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/state_t.hpp>
#include <common/messages/visualizer_log_t.hpp>
#include <gnc/State.hpp>

namespace maav
{
namespace tools
{
class MapDrawer
{
public:
    MapDrawer(YAML::Node config);

    void update(const visualizer_log_t& msg, bool use_estimator);
    void updateEstimator(const state_t* msg, bool use_estimator);
    void DrawMapPoints(const visualizer_log_t& msg);
    void DrawKeyFrames(const visualizer_log_t& msg, const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix& Twc);
    void DrawCovariance();
    void DrawVelocity();
    void SetCurrentCameraPose(const cv::Mat& Tcw, bool use_estimator);
    // void SetReferenceKeyFrame(KeyFrame* pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix& M);

private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
    maav::gnc::State state_;
};
}
}

#endif
