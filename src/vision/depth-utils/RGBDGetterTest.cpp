
#include "vision/core/RGBDPlayback.hpp"
#include <iostream>

using maav::vision::RGBDPlayback;

void test_getRGB(RGBDPlayback&);
void test_getDepth(RGBDPlayback&);
void test_getCombined(RGBDPlayback&);
void test_getCloud(RGBDPlayback&);
void test_getCloudXYZRGBA(RGBDPlayback&);

int main()
{
    RGBDPlayback rgbd(0);
    for (int i = 0; i < 10; ++i)
    {
        test_getRGB(rgbd);
        test_getDepth(rgbd);
        test_getCombined(rgbd);

        cv::waitKey(0);

        test_getCloud(rgbd);
        test_getCloudXYZRGBA(rgbd);
        ++rgbd;
    }

    return 0;
}

void test_getRGB(RGBDPlayback& rgbd)
{
    cv::Mat img = rgbd.getRGB();

    imshow("RGB", img);
    cv::moveWindow("RGB", 0, 0);
}

void test_getDepth(RGBDPlayback& rgbd)
{
    cv::Mat img = rgbd.getDepth();
    imshow("Depth", img);
    cv::moveWindow("Depth", 500, 0);
}

void test_getCombined(RGBDPlayback& rgbd)
{
    cv::Mat img = rgbd.getCombined();
    imshow("Combined", img);
}

void test_getCloud(RGBDPlayback& rgbd)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
        rgbd.getCloud();
    std::cout << cloud->size() << std::endl;
}
