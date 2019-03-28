#ifndef CAMERA_INPUT_BASE_HPP
#define CAMERA_INPUT_BASE_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace maav::vision
{

// Used to return pose data from the tracking camera
// Without having realsense types being returned
class CameraPoseData
{
public:
    float x_translation_;
    float y_translation_;
    float z_translation_;

    float x_velocity_;
    float y_velocity_;
    float z_velocity_;

    float x_acceleration_;
    float y_acceleration_;
    float z_acceleration_;

    float Qi_rotation_;
    float Qj_rotation_;
    float Qk_rotation_;
    float Qr_rotation_;

    float x_angular_velocity_;
    float y_angular_velocity_;
    float z_angular_velocity_;

    float x_angular_acceleration_;
    float y_angular_acceleration_;
    float z_angular_acceleration_;

    float tracker_confidence_;
    float mapper_confidence_;
};

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

    virtual void disableAutoExposure() = 0;

    /**
     * Loads the next frame into memory.
     */
    virtual bool loadNext() = 0;

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

    /**
     * Returns the pos tracking info if being published
     */

    virtual CameraPoseData getPoseData() = 0;

private:
    int tag_;
    static int global_tag_;
};
// CameraInterfaceBase
}

#endif
