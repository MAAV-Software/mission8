#include "Viewer.hpp"

#include <thread>

#include <opencv2/highgui.hpp>

#include <common/messages/MsgChannels.hpp>
#include <gnc/utils/ZcmConversion.hpp>

using namespace std::chrono_literals;

namespace maav
{
namespace tools
{
Viewer::Viewer(YAML::Node config, std::shared_ptr<FrameDrawer> frame_drawer,
    std::shared_ptr<MapDrawer> map_drawer, zcm::ZCM& zcm_node)
    : fps_(config["fps"].as<double>()),
      period_(static_cast<int>(1000.0 / fps_)),
      mViewpointX(config["Viewer.ViewpointX"].as<float>()),
      mViewpointY(config["Viewer.ViewpointY"].as<float>()),
      mViewpointZ(config["Viewer.ViewpointZ"].as<float>()),
      mViewpointF(config["Viewer.ViewpointF"].as<float>()),
      follow_(true),
      localization_mode_(false),
      finish_requested_(false),
      finished_(true),
      stop_requested_(false),
      stopped_(true),
      frame_drawer_(frame_drawer),
      map_drawer_(map_drawer)
{
    zcm_node.subscribe(maav::VISUALIZER_CHANNEL, &Viewer::updateLocalizer, this);
    zcm_node.subscribe(maav::STATE_CHANNEL, &Viewer::updateEstimator, this);
}

void Viewer::updateLocalizer(
    const zcm::ReceiveBuffer*, const std::string&, const visualizer_log_t* msg)
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    data = *msg;
    frame_drawer_->update(data);
    if (!(*menuKalmanState)) map_drawer_->update(data, false);
}

void Viewer::updateEstimator(const zcm::ReceiveBuffer*, const std::string&, const state_t* msg)
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (*menuKalmanState) map_drawer_->updateEstimator(msg, true);
}

void Viewer::Run()
{
    finished_ = false;
    stopped_ = false;

    // Define Camera Render Object (for view / scene browsing)
    createWindow();
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            drawWindow(s_cam, d_cam);
        }

        if (*menuReset)
        {
            reset();
        }

        if (Stop())
        {
            while (isStopped())
            {
                usleep(3000);
            }
        }

        if (CheckFinish())
        {
            break;
        }

        const cv::Mat& im = frame_drawer_->getDrawnFrame();
        cv::imshow(FORWARD_WINDOW_NAME, im);
        cv::waitKey(period_);
    }

    SetFinish();
    pangolin::DestroyWindow(PANGOLIN_WINDOW_NAME);
    cv::destroyWindow(FORWARD_WINDOW_NAME);
}

void Viewer::createWindow()
{
    pangolin::CreateWindowAndBind(PANGOLIN_WINDOW_NAME, 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    menuFollowCamera = std::make_unique<Checkbox>(Checkbox("menu.Follow Camera", true, true));
    menuShowPoints = std::make_unique<Checkbox>(Checkbox("menu.Show Points", true, true));
    menuShowKeyFrames = std::make_unique<Checkbox>(Checkbox("menu.Show KeyFrames", true, true));
    menuShowGraph = std::make_unique<Checkbox>(Checkbox("menu.Show Graph", true, true));
    menuLocalizationMode =
        std::make_unique<Checkbox>(Checkbox("menu.Localization Mode", false, true));
    menuKalmanState = std::make_unique<Checkbox>(Checkbox("menu.Kalman State", true, true));
    menuKalmanCovariance =
        std::make_unique<Checkbox>(Checkbox("menu.Kalman Covariance", false, true));
    menuKalmanVelocity = std::make_unique<Checkbox>(Checkbox("menu.Kalman Velocity", false, true));
    menuReset = std::make_unique<Checkbox>(Checkbox("menu.Reset", false, false));

    Twc.SetIdentity();

    // Start frame viewer
    cv::namedWindow(FORWARD_WINDOW_NAME);
}

void Viewer::drawWindow(pangolin::OpenGlRenderState& s_cam, pangolin::View& d_cam)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    map_drawer_->GetCurrentOpenGLCameraMatrix(Twc);

    // Check if we need to follow the camera
    if (menuFollowCamera && follow_)
    {
        s_cam.Follow(Twc);
    }
    else if (*menuFollowCamera && !follow_)
    {
        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
            mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
        s_cam.Follow(Twc);
        follow_ = true;
    }
    else if (!*menuFollowCamera && follow_)
    {
        follow_ = false;
    }

    // Check if localization mode is activated
    if (*menuLocalizationMode && !localization_mode_)
    {
        setLocalizationMode(true);
    }
    else if (!*menuLocalizationMode && localization_mode_)
    {
        setLocalizationMode(false);
    }

    if (!*menuKalmanState && *menuKalmanCovariance)
    {
        *menuKalmanCovariance = false;
    }
    if (!*menuKalmanState && *menuKalmanVelocity)
    {
        *menuKalmanVelocity = false;
    }

    if (*menuKalmanCovariance)
    {
        map_drawer_->DrawCovariance();
    }

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    map_drawer_->DrawCurrentCamera(Twc);
    if (*menuShowKeyFrames || menuShowGraph)
        map_drawer_->DrawKeyFrames(data, *menuShowKeyFrames, *menuShowGraph);
    if (*menuShowPoints) map_drawer_->DrawMapPoints(data);

    if (*menuKalmanVelocity)
    {
        map_drawer_->DrawVelocity();
    }

    glLineWidth(3);
    pangolin::glDrawAxis(10.0);

    glColor3f(0.9, 0.9, 0.9);
    glLineWidth(1);
    pangolin::glDraw_y0(0.1, 100);

    pangolin::FinishFrame();
}

void Viewer::setLocalizationMode(bool on)
{
    *menuLocalizationMode = on;
    localization_mode_ = on;
    // TODO: send zcm
}

void Viewer::reset()
{
    setLocalizationMode(false);
    *menuShowGraph = true;
    *menuShowKeyFrames = true;
    *menuShowPoints = true;
    follow_ = true;
    *menuFollowCamera = true;
    *menuReset = false;
}

void Viewer::RequestFinish()
{
    std::unique_lock<std::mutex> lock(finish_mutex_);
    finish_requested_ = true;
}

void Viewer::RequestStop()
{
    std::unique_lock<std::mutex> lock(stop_mutex_);
    if (!stopped_) stop_requested_ = true;
}

bool Viewer::isFinished()
{
    std::unique_lock<std::mutex> lock(finish_mutex_);
    return finished_;
}

bool Viewer::isStopped()
{
    std::unique_lock<std::mutex> lock(stop_mutex_);
    return stopped_;
}

void Viewer::Release()
{
    std::unique_lock<std::mutex> lock(stop_mutex_);
    stopped_ = false;
}

bool Viewer::Stop()
{
    std::unique_lock<std::mutex> lock_stop(stop_mutex_);
    std::unique_lock<std::mutex> lock_finish(finish_mutex_);

    if (finish_requested_)
    {
        return false;
    }
    else if (stop_requested_)
    {
        stopped_ = true;
        stop_requested_ = false;
        return true;
    }
    return false;
}

bool Viewer::CheckFinish()
{
    std::unique_lock<std::mutex> lock(finish_mutex_);
    return finish_requested_;
}

void Viewer::SetFinish()
{
    std::unique_lock<std::mutex> lock(finish_mutex_);
    finished_ = true;
}
}
}