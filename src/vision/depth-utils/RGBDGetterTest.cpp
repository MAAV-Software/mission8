
#include "vision/depth-utils/RGBDGetter.hpp"
#include <iostream>

void test_getRGB(RGBDGetter&);
void test_getDepth(RGBDGetter&);
void test_getCombined(RGBDGetter&);
void test_getCloud(RGBDGetter&);
void test_getCloudXYZRGBA(RGBDGetter&);

int main()
{
	RGBDGetter rgbd(0);
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

void test_getRGB(RGBDGetter& rgbd)
{
	cv::Mat img;

	rgbd.getRGB(img);
	imshow("RGB", img);
	cv::moveWindow("RGB", 0, 0);
}

void test_getDepth(RGBDGetter& rgbd)
{
	cv::Mat img;

	rgbd.getDepth(img);
	imshow("Depth", img);
	cv::moveWindow("Depth", 500, 0);
}

void test_getCombined(RGBDGetter& rgbd)
{
	cv::Mat img;

	rgbd.getCombined(img);
	imshow("Combined", img);
}

void test_getCloud(RGBDGetter& rgbd)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;

	rgbd.getCloud(cloud);
	std::cout << cloud.size() << std::endl;
}

void test_getCloudXYZRGBA(RGBDGetter& rgbd)
{
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

	rgbd.getCloudXYZRGBA(cloud);
	std::cout << cloud.size() << std::endl;
}
