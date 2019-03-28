#include "vision/core/CameraDriverHelper.hpp"

#include "common/messages/depth_image_t.hpp"
#include "common/messages/point_cloud_t.hpp"
#include "common/messages/point_t.hpp"
#include "common/messages/rgb_image_t.hpp"
#include "common/messages/camera_pose_t.hpp"

#include "vision/core/utilities.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <yaml-cpp/node/detail/bool_type.h>

#include <iostream>
#include <vector>

using maav::vision::wrapInDepthMat;
using maav::vision::wrapInRGBMat;
using std::string;
using std::thread;
using std::vector;
using std::chrono::milliseconds;
using namespace std::literals::chrono_literals;

using pcl::PointCloud;
using pcl::PointXYZ;

namespace maav::vision
{
const string CameraDriverHelper::FORMAT_IPC = "ipc";

CameraDriverHelper::CameraDriverHelper(YAML::Node config, const string& zcm_format,
    const string& rgbd_channel_in, const string& pointcloud_channel_in,
    const string& pose_channel_in)
    : enabled_(config["enabled"].as<bool>()),
      publish_rgbd_(config["publish_rgbd"].as<bool>()),
      publish_pc_(config["publish_pointcloud"].as<bool>()),
      publish_pose_(config["publish_pose"].as<bool>()),
      autoexposure_(config["enable_autoexposure"].as<bool>()),
      zcm_{zcm_format},
      camera_(config),
      running_(false),
      rgbd_channel_(rgbd_channel_in),
      pointcloud_channel_(pointcloud_channel_in),
      pose_channel_(pose_channel_in)
{
    if (!zcm_.good()) std::cout << "ZCM bad" << std::endl;
}

void CameraDriverHelper::beginRecording()
{
    running_ = true;

    publish_thread_ = thread(&CameraDriverHelper::publish, this);
}

void CameraDriverHelper::endRecording()
{
    if (running_)
    {
        running_ = false;
        publish_thread_.join();
    }
}

void CameraDriverHelper::publish()
{
    if (!enabled_) return;
    if (!autoexposure_) camera_.disableAutoExposure();
    while (running_)
    {
        if (camera_.loadNext())
        {
            if (publish_rgbd_) rgbdPublish();
            if (publish_pc_) pointcloudPublish();
            if (publish_pose_) posPublish();
        }
        std::this_thread::sleep_for(5ms);  // Maybe make this time configurable?
    }
}

void CameraDriverHelper::rgbdPublish()
{
    rgbd_image_t rgbd;

    rgbd.rgb_image.width = camera_.getStreamWidth();
    rgbd.rgb_image.height = camera_.getStreamHeight();
    rgbd.rgb_image.size = rgbd.rgb_image.width * rgbd.rgb_image.height * 3;
    const int8_t* raw_color = reinterpret_cast<const int8_t*>(camera_.getRawColor());
    rgbd.rgb_image.raw_image.assign(raw_color, raw_color + rgbd.rgb_image.size);

    rgbd.depth_image.width = camera_.getStreamWidth();
    rgbd.depth_image.height = camera_.getStreamHeight();
    rgbd.depth_image.size = rgbd.depth_image.width * rgbd.depth_image.height;
    const int16_t* raw_depth = reinterpret_cast<const int16_t*>(camera_.getRawDepth());
    rgbd.depth_image.raw_image.assign(raw_depth, raw_depth + rgbd.depth_image.size);

    rgbd.utime = camera_.getUTime();

    zcm_.publish(rgbd_channel_, &rgbd);
}

void CameraDriverHelper::pointcloudPublish()
{
    PointCloud<PointXYZ>::Ptr cloud;
    cloud = camera_.getPointCloudBasic();

    point_cloud_t pcd;
    pcd.size = static_cast<int>(cloud->size());
    pcd.point_cloud.reserve(cloud->size());

    for (PointXYZ& p : *cloud)
    {
        point_t np;
        np.x = p.x;
        np.y = p.y;
        np.z = p.z;

        pcd.point_cloud.push_back(np);
    }

    pcd.utime = camera_.getUTime();

    zcm_.publish(pointcloud_channel_, &pcd);
}

void CameraDriverHelper::posPublish()
{
    CameraPoseData data = camera_.getPoseData();

    camera_pose_t message;

    // Set everything in the message to what is in the retrieved data
    message.x_translation_ = data.x_translation_;
    message.y_translation_ = data.y_translation_;
    message.z_translation_ = data.z_translation_;

    message.x_velocity_ = data.x_velocity_;
    message.y_velocity_ = data.y_velocity_;
    message.z_velocity_ = data.z_velocity_;

    message.x_acceleration_ = data.x_acceleration_;
    message.y_acceleration_ = data.y_acceleration_;
    message.z_acceleration_ = data.z_acceleration_;

    message.Qi_rotation_ = data.Qi_rotation_;
    message.Qj_rotation_ = data.Qj_rotation_;
    message.Qk_rotation_ = data.Qk_rotation_;
    message.Qr_rotation_ = data.Qr_rotation_;

    message.x_angular_velocity_ = data.x_angular_velocity_;
    message.y_angular_velocity_ = data.y_angular_velocity_;
    message.z_angular_velocity_ = data.z_angular_velocity_;

    message.x_angular_acceleration_ = data.x_angular_acceleration_;
    message.y_angular_acceleration_ = data.y_angular_acceleration_;
    message.z_angular_acceleration_ = data.z_angular_acceleration_;

    message.tracker_confidence_ = data.tracker_confidence_;
    message.mapper_confidence_ = data.mapper_confidence_;

    message.utime = camera_.getUTime();

    zcm_.publish(pose_channel_, &message);
}

}  // namespace maav::vision
