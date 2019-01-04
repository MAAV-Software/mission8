#include <common/messages/MsgChannels.hpp>
#include <gnc/slam/VisualizerLink.hpp>

namespace maav
{
namespace gnc
{
namespace slam
{
VisualizerLink::VisualizerLink(const std::string& zcm_url, Map* map_data, bool send_images)
    : zcm{zcm_url}, map_(map_data), send_images_(send_images)
{
}
void VisualizerLink::updateTracking(Tracking* tracker)
{
    unique_lock<mutex> lock(tracking_mutex_);

    // Send tracking info + images
    if (send_images_)
    {
        convertRgbImage(tracker->rgb_im);
        convertDepthImage(tracker->depth_im);
    }
    else
    {
        last_frame.img.rgb_image.raw_image.resize(0);
        last_frame.img.depth_image.raw_image.resize(0);
        last_frame.img.rgb_image.size = 0;
        last_frame.img.depth_image.size = 0;
    }

    auto keypoints = tracker->mCurrentFrame.mvKeys;
    last_frame.num_keypoints = keypoints.size();
    size_t N = keypoints.size();

    last_frame.keypoints.resize(N);

    last_frame.only_tracking = tracker->mbOnlyTracking;

    // if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED)
    // {
    //     mvIniKeys = pTracker->mInitialFrame.mvKeys;
    //     mvIniMatches = pTracker->mvIniMatches;
    // }
    if (tracker->mLastProcessedState == Tracking::OK)
    {
        for (size_t i = 0; i < N; i++)
        {
            last_frame.keypoints[i].x = keypoints[i].pt.x;
            last_frame.keypoints[i].y = keypoints[i].pt.y;

            MapPoint* map_pt = tracker->mCurrentFrame.mvpMapPoints[i];
            if (map_pt)
            {
                if (!tracker->mCurrentFrame.mvbOutlier[i])
                {
                    if (map_pt->Observations() > 0)
                        last_frame.keypoints[i].info = 0 | KP_MAP_BIT;
                    else
                        last_frame.keypoints[i].info = 0 | KP_VO_BIT;
                }
            }
        }
    }

    last_frame.state = static_cast<int>(tracker->mLastProcessedState);
    last_frame.utime = tracker->last_frame_usec;

    // Send points
    const vector<MapPoint*>& vpMPs = map_->GetAllMapPoints();
    const vector<MapPoint*>& vpRefMPs = map_->GetReferenceMapPoints();
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    last_frame.num_points = vpMPs.size();
    last_frame.points.resize(last_frame.num_points);
    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
    {
        if (vpMPs[i]->isBad()) continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        last_frame.points[i].pos[0] = pos.at<float>(0);
        last_frame.points[i].pos[1] = pos.at<float>(1);
        last_frame.points[i].pos[2] = pos.at<float>(2);

        if (spRefMPs.count(vpMPs[i]))
        {
            last_frame.points[i].info = 0 | PT_REF_BIT;
        }
        else
        {
            last_frame.points[i].info = 0 | PT_MAP_BIT;
        }
    }

    // Send keyframes
    const vector<KeyFrame*> vpKFs = map_->GetAllKeyFrames();
    last_frame.num_keyframes = vpKFs.size();
    last_frame.keyframes.resize(last_frame.num_keyframes);

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        cv::Mat Twc = pKF->GetPoseInverse().t();
        for (size_t row = 0; row < 4; row++)
        {
            for (size_t col = 0; col < 3; col++)
            {
                last_frame.keyframes[i].pose[row][col] = Twc.at<float>(row, col);
            }
        }
    }

    // Send current pose
    cv::Mat pose_t = current_pose_.t();
    for (size_t row = 0; row < 4; row++)
    {
        for (size_t col = 0; col < 3; col++)
        {
            last_frame.current_pose.pose[row][col] = pose_t.at<float>(row, col);
        }
    }

    // if (bDrawGraph)
    // {
    //     glLineWidth(mGraphLineWidth);
    //     glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
    //     glBegin(GL_LINES);

    //     for (size_t i = 0; i < vpKFs.size(); i++)
    //     {
    //         // Covisibility Graph
    //         const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
    //         cv::Mat Ow = vpKFs[i]->GetCameraCenter();
    //         if (!vCovKFs.empty())
    //         {
    //             for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend =
    //             vCovKFs.end();
    //                  vit != vend; vit++)
    //             {
    //                 if ((*vit)->mnId < vpKFs[i]->mnId) continue;
    //                 cv::Mat Ow2 = (*vit)->GetCameraCenter();
    //                 glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
    //                 glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
    //             }
    //         }

    //         // Spanning tree
    //         KeyFrame* pParent = vpKFs[i]->GetParent();
    //         if (pParent)
    //         {
    //             cv::Mat Owp = pParent->GetCameraCenter();
    //             glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
    //             glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
    //         }

    //         // Loops
    //         set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
    //         for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end();
    //              sit != send; sit++)
    //         {
    //             if ((*sit)->mnId < vpKFs[i]->mnId) continue;
    //             cv::Mat Owl = (*sit)->GetCameraCenter();
    //             glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
    //             glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
    //         }
    //     }

    zcm.publish(maav::VISUALIZER_CHANNEL, &last_frame);
    zcm.flush();
}

void VisualizerLink::convertRgbImage(const cv::Mat& im)
{
    size_t len = im.rows * im.cols * im.elemSize1() * 3;
    im.elemSize();
    last_frame.img.rgb_image.raw_image.resize(len);

    last_frame.img.rgb_image.width = im.cols;
    last_frame.img.rgb_image.height = im.rows;
    last_frame.img.rgb_image.size = len;

    memcpy(last_frame.img.rgb_image.raw_image.data(), im.data, len);
}

void VisualizerLink::convertDepthImage(const cv::Mat& im)
{
    cv::Mat u16_depth;
    im.convertTo(u16_depth, CV_16U, 1000);
    size_t len = u16_depth.rows * u16_depth.cols;
    last_frame.img.depth_image.raw_image.resize(len);

    last_frame.img.depth_image.width = im.cols;
    last_frame.img.depth_image.height = im.rows;
    last_frame.img.depth_image.size = im.rows * im.cols;

    memcpy(
        last_frame.img.depth_image.raw_image.data(), u16_depth.data, len * u16_depth.elemSize1());
}

void VisualizerLink::setCurrentPose(const cv::Mat& pose)
{
    unique_lock<mutex> lock(tracking_mutex_);
    current_pose_ = pose;
}
}
}
}