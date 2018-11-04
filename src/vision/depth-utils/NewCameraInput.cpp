
#include "vision/depth-utils/NewCameraInput.hpp"

#include <stdlib.h>
#include <iostream>

using std::string;

NewCameraInput::NewCameraInput(string serial_number)
{
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_device(serial_number);
    rs2::pipeline_profile selection = pipe.start(cfg);

    // get intrinsics
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    depthIntrinsics = depth_stream.get_intrinsics();

    auto color_stream = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    colorIntrinsics = color_stream.get_intrinsics();

    // get the scale
    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    scale = sensor.get_depth_scale();

    // get the extrinsics (very hacky)
    rs2_error* memory = (rs2_error*)malloc(5000);
    rs2_stream_profile* depth = (rs2_stream_profile*)(&depth_stream);
    rs2_stream_profile* color = (rs2_stream_profile*)(&color_stream);
    rs2_get_extrinsics(depth, color, &depthToColor, &memory);

    // Set up the align object
    // rs2_stream align_to = find_stream_to_align(selection.get_streams());
    alignObject.reset(new rs2::align(RS2_STREAM_COLOR));
}

void NewCameraInput::getRGB(cv::Mat& img) const
{
    img = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)colorImage, cv::Mat::AUTO_STEP).clone();
}

void NewCameraInput::getDepth(cv::Mat& img) const
{
    img = cv::Mat(cv::Size(640, 480), CV_16SC1, (void*)depthImage, cv::Mat::AUTO_STEP).clone();
}

void NewCameraInput::loadNext()
{
    frames = pipe.wait_for_frames();
    auto processed = alignObject->process(frames);

    // Try to get the frames from processed
    rgbFrame = processed.first_or_default(RS2_STREAM_COLOR);
    depthFrame = processed.get_depth_frame();
    if (rgbFrame)
    {
        colorImage = (const uint16_t*)rgbFrame.get_data();
    }
    if (depthFrame)
    {
        depthImage = (const uint16_t*)depthFrame.get_data();
    }
}

NewCameraInput& NewCameraInput::operator++()
{
    loadNext();
    return *this;
}

void NewCameraInput::getPointCloudBasic(pcl::PointCloud<pcl::PointXYZ>& cloud) const
{
    cloud.clear();
    for (int dy{0}; dy < depthIntrinsics.height; ++dy)
    {
        for (int dx{0}; dx < depthIntrinsics.width; ++dx)
        {
            // Retrieve depth value and map it to more "real" coordinates
            uint16_t depthValue = depthImage[dy * depthIntrinsics.width + dx];
            float depthInMeters = depthValue * scale;
            // Skip over values with a depth of zero (not found depth)
            if (depthValue == 0) continue;

            // For mapping color to depth
            // Map from pixel coordinates in the depth image to pixel coordinates in the color image
            float depth_pixel[2] = {(float)dx, (float)dy};
            // Projects the depth value into 3-D space
            float depthPoint[3];
            rs2_deproject_pixel_to_point(depthPoint, &depthIntrinsics, depth_pixel, depthInMeters);

            cloud.push_back(pcl::PointXYZ(depthPoint[0], depthPoint[1], depthPoint[2]));
        }
    }
}

void NewCameraInput::getMappedPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) const
{
    cloud.clear();
    cloud = pcl::PointCloud<pcl::PointXYZ>(640, 480, pcl::PointXYZ(0, 0, 0));
    for (int dy{0}; dy < depthIntrinsics.height; ++dy)
    {
        for (int dx{0}; dx < depthIntrinsics.width; ++dx)
        {
            // Retrieve depth value and map it to more "real" coordinates
            uint16_t depthValue = depthImage[dy * depthIntrinsics.width + dx];
            float depthInMeters = depthValue * scale;
            // Skip over values with a depth of zero (not found depth)
            if (depthValue == 0) continue;
            // For mapping color to depth
            // Map from pixel coordinates in the depth image to pixel coordinates in the color image
            float depth_pixel[2] = {(float)dx, (float)dy};
            // Projects the depth value into 3-D space
            float depthPoint[3];
            rs2_deproject_pixel_to_point(depthPoint, &depthIntrinsics, depth_pixel, depthInMeters);
            cloud.at(dx, dy) = pcl::PointXYZ(depthPoint[0], depthPoint[1], depthPoint[2]);
        }
    }
}
