
#ifndef CAMERA_INPUT_BASE_HPP
#define CAMERA_INPUT_BASE_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CameraInputBase
{
public:
    CameraInputBase();

    virtual void getRGB(cv::Mat& mat) const = 0;

    virtual void getDepth(cv::Mat& mat) const = 0;

    virtual void loadNext() = 0;

    virtual CameraInputBase& operator++();

    virtual int getTag() const;

    virtual void getPointCloudBasic(pcl::PointCloud<pcl::PointXYZ>& cloud) const = 0;

    virtual void getMappedPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) const = 0;

    virtual ~CameraInputBase() = default;

private:
    int tag;
    static int GLOBAL_TAG;
};

#endif
