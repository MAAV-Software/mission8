#ifndef CAMERA_INPUT_BASE_HPP
#define CAMERA_INPUT_BASE_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace maav::vision
{
class CameraInterfaceBase
{
public:
    CameraInterfaceBase();

    virtual ~CameraInterfaceBase() = default;

    /**
     * Takes the recently loaded RGB frame and copies it into mat.
     */
    virtual cv::Mat getRGB() const = 0;

    /**
     * Takes the recently loaded Depth frame and copies it into mat.
     */
    virtual cv::Mat getDepth() const = 0;

    /**
     * Loads the next frame into memory.
     */
    virtual void loadNext() = 0;

    /**
     * Loads the next frame into memory (calls loadNext)
     */
    virtual CameraInterfaceBase& operator++();

    virtual int getTag() const;

    /**
     * Takes the recently loaded Point Cloud and copies it into mat.
     */
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudBasic() const = 0;

    /**
     * Takes the recently loaded mapped Point Cloud and copies it into mat.
     */
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getMappedPointCloud() const = 0;

private:
    int tag_;
    static int global_tag_;
};
// CameraInterfaceBase
}

#endif
