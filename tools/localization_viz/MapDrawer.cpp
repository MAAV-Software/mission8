#include "MapDrawer.hpp"
#include <gnc/slam/VisualizerLink.hpp>

#include <set>
#include <vector>

#include <gnc/utils/ZcmConversion.hpp>

using maav::gnc::slam::PT_MAP_BIT;
using maav::gnc::slam::PT_REF_BIT;

namespace maav
{
namespace tools
{
MapDrawer::MapDrawer(YAML::Node config)
    : mKeyFrameSize(config["Viewer.KeyFrameSize"].as<float>()),
      mKeyFrameLineWidth(config["Viewer.KeyFrameLineWidth"].as<float>()),
      mGraphLineWidth(config["Viewer.GraphLineWidth"].as<float>()),
      mPointSize(config["Viewer.PointSize"].as<float>()),
      mCameraSize(config["Viewer.CameraSize"].as<float>()),
      mCameraLineWidth(config["Viewer.CameraLineWidth"].as<float>()),
      state_(gnc::State::zero(0))
{
}

void MapDrawer::update(const visualizer_log_t& msg, bool use_estimator)
{
    cv::Mat Twc(4, 4, CV_32F);
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            Twc.at<float>(row, col) = msg.current_pose.pose[row][col];
        }
    }
    Twc.at<float>(0, 3) = 0;
    Twc.at<float>(1, 3) = 0;
    Twc.at<float>(2, 3) = 0;
    Twc.at<float>(3, 3) = 1;

    if (!use_estimator) SetCurrentCameraPose(Twc.t(), false);
}

void MapDrawer::updateEstimator(const state_t* msg, bool use_estimator)
{
    state_ = gnc::ConvertState(*msg);
    cv::Mat Twc(4, 4, CV_32F);
    if (use_estimator) SetCurrentCameraPose(Twc, true);
}

void MapDrawer::SetCurrentCameraPose(const cv::Mat& Tcw, bool use_estimator)
{
    std::unique_lock<std::mutex> lock(mMutexCamera);
    if (use_estimator)
    {
        mCameraPose = Tcw.clone();
        const Eigen::Quaterniond& q = state_.attitude().unit_quaternion();
        Eigen::Quaterniond q_v(q.w(), -q.y(), -q.z(), -q.x());
        Eigen::Vector3d p_v(-state_.position().y(), -state_.position().z(), -state_.position().x());
        Eigen::Matrix3d mat_v = q_v.matrix();
        for (size_t i = 0; i < 3; i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                mCameraPose.at<float>(i, j) = mat_v(i, j);
            }
            mCameraPose.at<float>(i, 3) = p_v(i);
        }
        mCameraPose.at<float>(3, 3) = 1;
    }
    else
    {
        mCameraPose = Tcw.clone();
    }
}

void MapDrawer::DrawMapPoints(const visualizer_log_t& msg)
{
    if (msg.points.empty()) return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (int i = 0; i < msg.num_points; i++)
    {
        if (msg.points[i].info & PT_REF_BIT) continue;
        if (msg.points[i].info & PT_MAP_BIT)
            glVertex3f(msg.points[i].pos[0], msg.points[i].pos[1], msg.points[i].pos[2]);
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (int i = 0; i < msg.num_points; i++)
    {
        if (msg.points[i].info & PT_MAP_BIT) continue;
        if (msg.points[i].info & PT_REF_BIT)
            glVertex3f(msg.points[i].pos[0], msg.points[i].pos[1], msg.points[i].pos[2]);
    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(
    const visualizer_log_t& msg, const bool bDrawKF, const bool bDrawGraph)
{
    const float& w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    for (const auto& kf : msg.keyframes)
    {
        cv::Mat Twc(4, 4, CV_32F);
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                Twc.at<float>(row, col) = kf.pose[row][col];
            }
        }
        Twc.at<float>(0, 3) = 0;
        Twc.at<float>(1, 3) = 0;
        Twc.at<float>(2, 3) = 0;
        Twc.at<float>(3, 3) = 1;

        glPushMatrix();

        glMultMatrixf(Twc.ptr<GLfloat>(0));

        glLineWidth(mKeyFrameLineWidth);
        glColor3f(0.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }
}

void MapDrawer::DrawVelocity()
{
    Eigen::Vector3d gl_position;
    gl_position.x() = state_.position().y();
    gl_position.y() = state_.position().z();
    gl_position.z() = state_.position().x();

    Eigen::Vector3d gl_velocity;
    gl_velocity.x() = state_.velocity().y();
    gl_velocity.y() = state_.velocity().z();
    gl_velocity.z() = state_.velocity().x();

    glLineWidth(1.5);
    glColor3f(1.0, 0, 0);
    pangolin::glDrawLine(gl_position.x(), gl_position.y(), gl_position.z(),
        gl_position.x() + gl_velocity.x(), gl_position.y() + gl_velocity.y(),
        gl_position.z() + gl_velocity.z());
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix& Twc)
{
    const float& w = mCameraSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();
}

void DrawEllipsoid(/*float x, float y, float z, float sx, float sy, float sz*/)
{
    // glBegin();
    auto* quadric = gluNewQuadric();
    glColor3f(1, 0, 0);
    glTranslatef(2, 2, 2);
    gluSphere(quadric, 100, 100, 100);
    glEnd();
}

void MapDrawer::DrawCovariance()
{
    Eigen::Matrix3d position_covar = state_.covariance().block<3, 3>(3, 3);
    Eigen::Matrix3d t_pos_cov;
    t_pos_cov(0, 0) = position_covar(1, 1);
    t_pos_cov(0, 1) = position_covar(1, 2);
    t_pos_cov(0, 2) = position_covar(1, 0);
    t_pos_cov(1, 0) = position_covar(2, 1);
    t_pos_cov(1, 1) = position_covar(2, 2);
    t_pos_cov(1, 2) = position_covar(2, 0);
    t_pos_cov(2, 0) = position_covar(0, 1);
    t_pos_cov(2, 1) = position_covar(0, 2);
    t_pos_cov(2, 2) = position_covar(0, 0);

    DrawEllipsoid();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix& M)
{
    if (!mCameraPose.empty())
    {
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);
        {
            std::unique_lock<std::mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
            twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
        }

        M.m[0] = Rwc.at<float>(0, 0);
        M.m[1] = Rwc.at<float>(1, 0);
        M.m[2] = Rwc.at<float>(2, 0);
        M.m[3] = 0.0;

        M.m[4] = Rwc.at<float>(0, 1);
        M.m[5] = Rwc.at<float>(1, 1);
        M.m[6] = Rwc.at<float>(2, 1);
        M.m[7] = 0.0;

        M.m[8] = Rwc.at<float>(0, 2);
        M.m[9] = Rwc.at<float>(1, 2);
        M.m[10] = Rwc.at<float>(2, 2);
        M.m[11] = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15] = 1.0;
    }
    else
    {
        M.SetIdentity();
    }
}
}
}