
#include "vision/depth-utils/RGBDGetter.hpp"

#include <string>

using namespace pf;

const std::string FILE_PATH = "build/src/vision/depth-utils/";

RGBDGetter::RGBDGetter(int _num) : dir_num(_num)
{
    timestamps.open(FILE_PATH + "RGBD_DATA/timestamps.txt");
    timestamps >> timestamp;
}

void RGBDGetter::getRGB(cv::Mat &img) { openMatFromDirectory("RGB", img); }
void RGBDGetter::getDepth(cv::Mat &img) { openMatFromDirectory("DepthImage", img); }
void RGBDGetter::getCombined(cv::Mat &img) { openMatFromDirectory("Combined", img); }
void RGBDGetter::getCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    std::string imgName = "RGBD_DATA/Cloud";
    imgName += std::to_string(dir_num);
    imgName += "/";
    imgName += std::to_string(counter);
    imgName += ".pcd";
    pcl::io::loadPCDFile(FILE_PATH + imgName, cloud);
}

void RGBDGetter::getCloud(std::vector<Point3f> &cloud)
{
    pcl::PointCloud<pcl::PointXYZ> points;
    getCloud(points);
    for (pcl::PointXYZ point : points)
    {
        Point3f p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        cloud.push_back(p);
    }
}

void RGBDGetter::getCloudXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
    std::string imgName = "RGBD_DATA/RCloud";
    imgName += std::to_string(dir_num);
    imgName += "/";
    imgName += std::to_string(counter);
    imgName += ".pcd";
    pcl::io::loadPCDFile(FILE_PATH + imgName, cloud);
}

RGBDGetter &RGBDGetter::operator++()
{
    counter++;
    timestamps >> timestamp;
    return *this;
}

void RGBDGetter::openMatFromDirectory(std::string dir, cv::Mat &img)
{
    std::string imgName = "RGBD_DATA/";
    imgName += dir;
    imgName += std::to_string(dir_num);
    imgName += "/";
    imgName += std::to_string(counter);
    imgName += ".png";
    std::cout << imgName << std::endl;
    img = cv::imread(FILE_PATH + imgName);
    if (!img.data)
        std::cout << "did not work" << std::endl;
    else
        std::cout << "worked" << std::endl;
}
