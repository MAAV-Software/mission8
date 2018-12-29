#include "vision/depth-utils/RGBDPlayback.hpp"

using namespace maav::vision;

using std::string;
using std::to_string;
using std::runtime_error;

const std::string DEFAULT_PATH = "build/src/vision/depth-utils/";

RGBDPlayback::RGBDPlayback(int num, const string& path) : file_path_(path)
{
    dir_num_ = to_string(num);
    timestamps_.open(file_path_ + "RGBD_DATA/timestamps_.txt");
    timestamps_ >> timestamp_;
}

RGBDPlayback::RGBDPlayback(int num) : RGBDPlayback(num, DEFAULT_PATH) {}
cv::Mat RGBDPlayback::getRGB()
{
    cv::Mat img;
    openMatFromDirectory("RGB", img);
    return img;
}

cv::Mat RGBDPlayback::getDepth()
{
    cv::Mat img;
    openMatFromDirectory("DepthImage", img);
    return img;
}

cv::Mat RGBDPlayback::getCombined()
{
    cv::Mat img;
    openMatFromDirectory("Combined", img);
    return img;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDPlayback::getCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // construct the path to the cloud
    std::string img_name = "RGBD_DATA/Cloud";
    img_name += dir_num_;
    img_name += "/";
    img_name += to_string(counter_);
    img_name += ".pcd";
    pcl::io::loadPCDFile(file_path_ + img_name, *cloud);
    return cloud;
}

RGBDPlayback& RGBDPlayback::operator++()
{
    ++counter_;
    timestamps_ >> timestamp_;
    return *this;
}

void RGBDPlayback::openMatFromDirectory(const string& dir, cv::Mat& img)
{
    // construct the path of the image
    std::string img_name = "RGBD_DATA/";
    img_name += dir;
    img_name += dir_num_;
    img_name += "/";
    img_name += to_string(counter_);
    img_name += ".png";
    img = cv::imread(file_path_ + img_name);
    if (!img.data) throw runtime_error("Bad File: " + img_name);
}
