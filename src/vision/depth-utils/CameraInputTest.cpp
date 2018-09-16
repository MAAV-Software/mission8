#include <iostream>
#include "vision/depth-utils/CameraInput.hpp"

void test_getRGB(CameraInput&);
void test_getDepth(CameraInput&);
void test_getCombined(CameraInput&);
void test_getCloud(CameraInput&);
void test_getCloudXYZRGBA(CameraInput&);
void test_getCloudXYZRGB(CameraInput&);

int main() {

	CameraInput cam(0);
	for (int i = 0; i < 10; ++i) {
		test_getRGB(cam);
		test_getDepth(cam);
		test_getCombined(cam);

		cv::waitKey(0);

		test_getCloud(cam);
		test_getCloudXYZRGB(cam);
		test_getCloudXYZRGBA(cam);
		++cam;
	}

	return 0;
}

void test_getRGB(CameraInput& cam) {
	cv::Mat img;

	cam.getRGB(img);
	std::cout << img.empty() << std::endl;
	imshow("RGB", img);
	cv::moveWindow("RGB", 0, 0);
}

void test_getDepth(CameraInput& cam) {
	cv::Mat img;
	cam.getDepth(img);
	imshow("Depth", img);
	cv::moveWindow("Depth", 500, 0);
}

void test_getCombined(CameraInput& cam) {
	cv::Mat img;

	cam.getCombined(img);
	imshow("Combined", img);
}

void test_getCloud(CameraInput& cam) {
	pcl::PointCloud<pcl::PointXYZ> cloud;

	cam.getCloud(cloud);
	std::cout << cloud.size() << std::endl;
}

void test_getCloudXYZRGB(CameraInput& cam) {
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	cam.getCloudXYZRGB(cloud);
	std::cout << cloud.size() << std::endl;
}

void test_getCloudXYZRGBA(CameraInput& cam) {
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

	cam.getCloudXYZRGBA(cloud);
	std::cout << cloud.size() << std::endl;
}

