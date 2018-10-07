#include "vision/depth-utils/CameraInput.hpp"

#include <string>

using namespace pf;

CameraInput::CameraInput(int _id) : CameraInput(_id, new rs::context, true) {}
CameraInput::CameraInput(int _id, rs::context *ctx_in) : CameraInput(_id, ctx_in, false) {}
CameraInput::CameraInput(int _id, rs::context *ctx_in, bool sourceIn)
	: camera_id{_id}, isSource{sourceIn}
{
	std::cout << ctx_in << "\n";
	rs::log_to_console(rs::log_severity::warn);
	ctx = ctx_in;
	printf("There are %d connected realsense devices.\n", ctx->get_device_count());
	printf("Attempting to pull data from device #%d", camera_id);
	if (ctx->get_device_count() < 1)
	{
		throw std::string("");
	}
	devicePtr = ctx->get_device(camera_id);
	devicePtr->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
	devicePtr->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 60);
	devicePtr->start();
	loadNext();
}

CameraInput::~CameraInput()
{
	if (isSource) delete ctx;
}

rs::context *CameraInput::getContext() { return ctx; }
void CameraInput::getRGB(cv::Mat &img) const
{
	img = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)colorImage, cv::Mat::AUTO_STEP).clone();
}

void CameraInput::getDepth(cv::Mat &img) const
{
	img = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *)depthImage, cv::Mat::AUTO_STEP).clone();
}

void CameraInput::getCombined(cv::Mat &img)
{
	cv::Mat color(cv::Size(640, 480), CV_8UC3, (void *)colorImage, cv::Mat::AUTO_STEP);
	cv::Mat depthMat(cv::Size(640, 480), CV_16UC1, (void *)depthImage, cv::Mat::AUTO_STEP);
	cv::Mat tempDepthMat;
	depthMat.convertTo(tempDepthMat, CV_8UC1);
	cv::Mat splitMats[4];
	cv::split(color, splitMats);
	cv::split(tempDepthMat, splitMats + 3);
	cv::merge(splitMats, 4, img);
	img.reshape(1, 1920);
	img = img.clone();
}

void CameraInput::getCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
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
			rs::float2 depth_pixel = {(float)dx, (float)dy};
			// Projects the depth value into 3-D space
			rs::float3 depthPoint = depthIntrinsics.deproject(depth_pixel, depthInMeters);

			cloud.push_back(pcl::PointXYZ(depthPoint.x, depthPoint.y, depthPoint.z));
		}
	}
}

void CameraInput::getCloud(std::vector<Point3f> &cloud)
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
			rs::float2 depth_pixel = {(float)dx, (float)dy};
			// Projects the depth value into 3-D space
			rs::float3 depthPoint = depthIntrinsics.deproject(depth_pixel, depthInMeters);
			Point3f p;
			p.x = depthPoint.x;
			p.y = depthPoint.y;
			p.z = depthPoint.z;
			cloud.push_back(p);
		}
	}
}

void CameraInput::getCloudXYZRGB(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
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
			rs::float2 depth_pixel = {(float)dx, (float)dy};
			// Projects the depth value into 3-D space
			rs::float3 depthPoint = depthIntrinsics.deproject(depth_pixel, depthInMeters);
			// Creates a corresponding color points in 3-D space (maybe)
			rs::float3 color_point = depth_to_color.transform(depthPoint);
			// Projects the 3-D color point into image space (2-D)
			rs::float2 color_pixel = color_intrin.project(color_point);
			const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
			// int colorIndex = cy * color_intrin.width + cx;
			int colorIndex = (cy * color_intrin.width + cx) * 3;
			// int channelSize = color_intrin.width * color_intrin.height;
			pcl::PointXYZRGB xyzrgbPoint;
			if (cy > color_intrin.height || cx > color_intrin.width || cy < 0 || cx < 0)
			{
				xyzrgbPoint.r = 255;
				xyzrgbPoint.g = 255;
				xyzrgbPoint.b = 255;
			}
			else
			{
				xyzrgbPoint.r = colorImage[colorIndex];
				xyzrgbPoint.g = colorImage[colorIndex + 1];
				xyzrgbPoint.b = colorImage[colorIndex + 2];
			}
			xyzrgbPoint.x = depthPoint.x;
			xyzrgbPoint.y = depthPoint.y;
			xyzrgbPoint.z = depthPoint.z;
			cloud.push_back(xyzrgbPoint);
		}
	}
}

void CameraInput::getCloudXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
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
			rs::float2 depth_pixel = {(float)dx, (float)dy};
			// Projects the depth value into 3-D space
			rs::float3 depthPoint = depthIntrinsics.deproject(depth_pixel, depthInMeters);
			// Creates a corresponding color points in 3-D space (maybe)
			rs::float3 color_point = depth_to_color.transform(depthPoint);
			// Projects the 3-D color point into image space (2-D)
			rs::float2 color_pixel = color_intrin.project(color_point);
			const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
			// int colorIndex = cy * color_intrin.width + cx;
			int colorIndex = (cy * color_intrin.width + cx) * 3;
			// int channelSize = color_intrin.width * color_intrin.height;
			pcl::PointXYZRGBA xyzrgbaPoint;
			if (cy > color_intrin.height || cx > color_intrin.width || cy < 0 || cx < 0)
			{
				xyzrgbaPoint.r = 255;
				xyzrgbaPoint.g = 255;
				xyzrgbaPoint.b = 255;
				xyzrgbaPoint.a = 255;
			}
			else
			{
				xyzrgbaPoint.r = colorImage[colorIndex];
				xyzrgbaPoint.g = colorImage[colorIndex + 1];
				xyzrgbaPoint.b = colorImage[colorIndex + 2];
				xyzrgbaPoint.a = 255;
			}
			xyzrgbaPoint.x = depthPoint.x;
			xyzrgbaPoint.y = depthPoint.y;
			xyzrgbaPoint.z = depthPoint.z;
			cloud.push_back(xyzrgbaPoint);
		}
	}
}

void CameraInput::loadNext()
{
	devicePtr->wait_for_frames();
	depthImage = (const uint16_t *)devicePtr->get_frame_data(rs::stream::depth);
	colorImage = (const uint8_t *)devicePtr->get_frame_data(rs::stream::color);

	scale = devicePtr->get_depth_scale();
	depthIntrinsics = devicePtr->get_stream_intrinsics(rs::stream::depth);
	depth_to_color = devicePtr->get_extrinsics(rs::stream::depth, rs::stream::color);
	color_intrin = devicePtr->get_stream_intrinsics(rs::stream::color);
}

CameraInput &CameraInput::operator++()
{
	loadNext();
	return *this;
}

int CameraInput::getCamID() const { return camera_id; }
// not implemented
void CameraInput::getPointCloudBasic(pcl::PointCloud<pcl::PointXYZ> &cloud) const {}
// not implemented
void CameraInput::getMappedPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) const {}
