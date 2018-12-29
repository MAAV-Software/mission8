#ifndef LEGACY_CAMERA_INTERFACE_HPP_MAAV_VISION__
#define LEGACY_CAMERA_INTERFACE_HPP_MAAV_VISION__

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <librealsense/rs.hpp>

#include <cstdio>
#include <memory>
#include <vector>

#include "CameraInterfaceBase.hpp"

namespace maav::vision
{
// pulls data frame-by-frame directly from the camera
// and provides the RGBD data in various forms
class LegacyCameraInterface : public CameraInterfaceBase
{
public:
    /**
     * Creates a new LegacyCameraInterface instance that pulls
     * frames from the camera associated with the provided
     * camera id (creates a new rs::context to find the device)
     *
     * id : the camera id for the wanted camera
     */
    explicit LegacyCameraInterface(int id);

    /**
     * Creates a new LegacyCameraInterface instance that pulls
     * frames from the camera associated with the provided
     * camera id (using the given rs::context to find the device)
     *
     * id : the camera id for the wanted camera
     * ctx_in : the rs::context for the camera
     */
    LegacyCameraInterface(int id, std::shared_ptr<rs::context> ctxIn);

    /**
     * A LegacyCameraInterface instance cannot be created without
     * at least a given camera id
     */
    LegacyCameraInterface() = delete;

    /**
     * Deletes the rs::context if the LegacyCameraInterface instance
     * is a source
     */
    ~LegacyCameraInterface();

    /**
     * Returns the rs::context
     */
    std::shared_ptr<rs::context> getContext() const;

    /**
     * pulls the RGB data for the current camera frame
     * and provides it in a cv::Mat
     *
     * img : a cv::Mat reference to hold the returned RGB data
     */
    virtual cv::Mat getRGB() const override;

    /**
     * pulls the Depth data for the current camera frame
     * and provides it in a cv::Mat
     *
     * img : a cv::Mat reference to hold the returned depth data
     */
    virtual cv::Mat getDepth() const override;

    /**
     * pulls the combined RGB & Depth (RGBD) data for the
     * current camera frame and provides it in a cv::Mat
     *
     * img : a cv::Mat reference to hold the returned RGBD data
     */
    cv::Mat getCombined() const;

    /**
     * increments to the next frame using loadNext
     * Returns the LegacyCameraInterface instance following the increment
     */
    virtual void loadNext() override;

    /**
     * increments to the next frame using loadNext
     * Returns the LegacyCameraInterface instance following the increment
     */
    virtual LegacyCameraInterface& operator++() override;

    /**
     * returns the id associated with the camera
     */
    int getCamID() const;

    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudBasic() const override;

    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getMappedPointCloud() const override;

private:
    int camera_id_;

    rs::device* device_ptr_;
    std::shared_ptr<rs::context> ctx_;
    const uint16_t* depth_image_;
    const uint8_t* color_image_;

    float scale_;
    rs::intrinsics depth_intrinsics_;
    rs::extrinsics depth_to_color_;
    rs::intrinsics color_intrin_;
};
// LegacyCameraInterface
}

#endif
