
#ifndef NEW_CAMERA_INPUT_HPP
#define NEW_CAMERA_INPUT_HPP

#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>
#include <memory>
#include <string>
#include <vector>
#include "CameraInputBase.hpp"
#include "Point3f.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// pulls data frame-by-frame directly from the camera
// and provides the RGBD data in various forms
class NewCameraInput : public CameraInputBase
{
public:
    // A CameraInput instance cannot be created without
    // at least a given camera id
    NewCameraInput() = delete;

    // Creates new instance that pulls data
    // from the camera with the given serial number
    explicit NewCameraInput(std::string serial_number);

    // pulls the RGB data for the current camera frame
    // and provides it in a cv::Mat
    //
    // img : a cv::Mat reference to hold the returned RGB data
    virtual void getRGB(cv::Mat &img) const override;

    // pulls the Depth data for the current camera frame
    // and provides it in a cv::Mat
    //
    // img : a cv::Mat reference to hold the returned depth data
    virtual void getDepth(cv::Mat &img) const override;

    virtual void getPointCloudBasic(pcl::PointCloud<pcl::PointXYZ> &cloud) const override;

    virtual void getMappedPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) const override;

    // increments the camera to the next available frame
    // all the get methods now pull data from the most recent frame
    virtual void loadNext() override;

    // increments to the next frame using loadNext
    // Returns the CameraInput instance following the increment
    virtual NewCameraInput &operator++() override;

private:
    const uint16_t *depthImage;
    const uint16_t *colorImage;

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frames;
    rs2::frame rgbFrame;
    rs2::frame depthFrame;
    rs2_intrinsics depthIntrinsics;
    rs2_extrinsics depthToColor;
    rs2_intrinsics colorIntrinsics;
    std::unique_ptr<rs2::align> alignObject;
    float scale;
};

#endif
