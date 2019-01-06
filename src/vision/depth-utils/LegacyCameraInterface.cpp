#include "vision/depth-utils/LegacyCameraInterface.hpp"
#include "vision/depth-utils/RealsenseSettings.hpp"

#include <stdexcept>
#include <string>

using maav::vision::LegacyCameraInterface;

using std::cout;
using std::vector;
using std::string;
using std::runtime_error;
using std::shared_ptr;
using std::make_shared;

using cv::Mat;
using cv::Size;
using cv::split;
using cv::merge;

using rs::context;
using rs::stream;
using rs::format;
using rs::float2;
using rs::float3;
using rs::log_to_console;
using rs::log_severity;

using pcl::PointCloud;
using pcl::PointXYZ;

LegacyCameraInterface::LegacyCameraInterface(int id)
    : LegacyCameraInterface(id, make_shared<context>())
{
}

LegacyCameraInterface::LegacyCameraInterface(int id, shared_ptr<context> ctx_in)
    : camera_id_{id}, ctx_{ctx_in}
{
    cout << ctx_in << "\n";
    log_to_console(log_severity::warn);
    cout << "There are " << ctx_->get_device_count() << " connected realsense devices.\n";
    cout << "Attempting to pull data from device #" << camera_id_ << "\n";
    if (ctx_->get_device_count() < 1)
    {
        throw runtime_error("No connected cameras.");
    }
    device_ptr_ = ctx_->get_device(camera_id_);
    device_ptr_->enable_stream(stream::depth, STREAM_WIDTH, STREAM_HEIGHT, format::z16, STREAM_FPS);
    device_ptr_->enable_stream(
        stream::color, STREAM_WIDTH, STREAM_HEIGHT, format::bgr8, STREAM_FPS);
    device_ptr_->start();
    loadNext();
}

LegacyCameraInterface::~LegacyCameraInterface() {}
shared_ptr<context> LegacyCameraInterface::getContext() const { return ctx_; }
Mat LegacyCameraInterface::getRGB() const
{
    void* img_data = const_cast<void*>(static_cast<const void*>(color_image_));
    return Mat(Size(STREAM_WIDTH, STREAM_HEIGHT), CV_8UC3, img_data, Mat::AUTO_STEP).clone();
}

Mat LegacyCameraInterface::getDepth() const
{
    void* img_data = const_cast<void*>(static_cast<const void*>(depth_image_));
    return Mat(Size(STREAM_WIDTH, STREAM_HEIGHT), CV_16UC1, img_data, Mat::AUTO_STEP).clone();
}

Mat LegacyCameraInterface::getCombined() const
{
    Mat img;
    void* color_data = const_cast<void*>(static_cast<const void*>(color_image_));
    void* depth_data = const_cast<void*>(static_cast<const void*>(depth_image_));

    Mat color(Size(STREAM_WIDTH, STREAM_HEIGHT), CV_8UC3, color_data, Mat::AUTO_STEP);
    Mat depth_mat(Size(STREAM_WIDTH, STREAM_HEIGHT), CV_16UC1, depth_data, Mat::AUTO_STEP);
    Mat temp_depth_mat;
    depth_mat.convertTo(temp_depth_mat, CV_8UC1);
    Mat split_mats[4];
    split(color, split_mats);
    split(temp_depth_mat, split_mats + 3);
    merge(split_mats, 4, img);
    img.reshape(1, 1920);
    return img.clone();
}

void LegacyCameraInterface::disableAutoExposure()
{
    throw std::runtime_error("LegacyCameraInterface::disableAutoExposure isn't implemented");
}

bool LegacyCameraInterface::loadNext()
{
    device_ptr_->wait_for_frames();
    depth_image_ = static_cast<const uint16_t*>(device_ptr_->get_frame_data(stream::depth));
    color_image_ = static_cast<const uint8_t*>(device_ptr_->get_frame_data(stream::color));

    scale_ = device_ptr_->get_depth_scale();
    depth_intrinsics_ = device_ptr_->get_stream_intrinsics(stream::depth);
    depth_to_color_ = device_ptr_->get_extrinsics(stream::depth, stream::color);
    color_intrin_ = device_ptr_->get_stream_intrinsics(stream::color);
    // TODO: Don't just return false or delete this file
    return false;
}

LegacyCameraInterface& LegacyCameraInterface::operator++()
{
    loadNext();
    return *this;
}

int LegacyCameraInterface::getCamID() const { return camera_id_; }
// NOTE: Doesn't provide aligned point cloud for legacy cameras
// TODO Provide point aligned cloud, not sure if necessary
// Get back to me on that later
PointCloud<PointXYZ>::Ptr LegacyCameraInterface::getPointCloudBasic() const
{
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    for (int dy = 0; dy < depth_intrinsics_.height; ++dy)
    {
        for (int dx = 0; dx < depth_intrinsics_.width; ++dx)
        {
            // Retrieve depth value and map it to more "real" coordinates
            uint16_t depth_value = depth_image_[(dy * depth_intrinsics_.width) + dx];
            float depth_in_meters = depth_value * scale_;
            // Skip over values with a depth of zero (not found depth)
            if (depth_value == 0) continue;
            // For mapping color to depth
            // Map from pixel coordinates in the depth image to pixel
            // coordinates in the color image
            float2 depth_pixel = {static_cast<float>(dx), static_cast<float>(dy)};
            // Projects the depth value into 3-D space
            float3 depth_point = depth_intrinsics_.deproject(depth_pixel, depth_in_meters);
            cloud->push_back(PointXYZ(depth_point.x, depth_point.y, depth_point.z));
        }
    }
    return cloud;
}

// not implemented
PointCloud<PointXYZ>::Ptr LegacyCameraInterface::getMappedPointCloud() const
{
    throw runtime_error("LegacyCameraInterface::getMappedPointCloud isn't implemented");
    return PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
}
