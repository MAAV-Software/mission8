#include "vision/depth-utils/utilities.hpp"

#include <vector>

using cv::Mat;
using std::vector;

cv::Mat maav::vision::wrapInRGBMat(void* mem_ptr, int width, int height)
{
    return cv::Mat(cv::Size(width, height), CV_8UC3, mem_ptr, Mat::AUTO_STEP).clone();
}

cv::Mat maav::vision::wrapInDepthMat(void* mem_ptr, int width, int height)
{
    return cv::Mat(cv::Size(width, height), CV_16UC1, mem_ptr, Mat::AUTO_STEP).clone();
}

void maav::vision::rgbdToZcmType(const cv::Mat& rgb, const cv::Mat& depth, rgbd_image_t& zcm_img)
{
    rgb_image_t rgb_part;
    depth_image_t depth_part;

    rgb_part.width = rgb.cols;
    rgb_part.height = rgb.rows;
    rgb_part.size = rgb_part.width * rgb_part.height * 3;
    rgb_part.raw_image = vector<int8_t>(rgb.data, rgb.data + rgb_part.size);

    depth_part.width = depth.cols;
    depth_part.height = depth.rows;
    depth_part.size = depth_part.width * depth_part.height;
    depth_part.raw_image.assign((int16_t*)depth.datastart, (int16_t*)depth.dataend);

    zcm_img.rgb_image = rgb_part;
    zcm_img.depth_image = depth_part;
}

void maav::vision::zcmTypeToRgbd(const rgbd_image_t& zcm_img, cv::Mat& rgb, cv::Mat& depth)
{
    rgb = wrapInRGBMat((void*)(&zcm_img.rgb_image.raw_image[0]), 640, 480);
    depth = wrapInDepthMat((void*)(&zcm_img.depth_image.raw_image[0]), 640, 480);
}
