#ifndef NEW_CAMERA_INPUT_HPP
#define NEW_CAMERA_INPUT_HPP

#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>

#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>

#include "CameraInterfaceBase.hpp"

namespace maav::vision
{
/**
 * pulls data frame-by-frame directly from the camera
 * and provides the RGBD data in various forms
 */
class D400CameraInterface : public CameraInterfaceBase
{
public:
    /**
     * A LegacyCameraInterface instance cannot be created without
     * at least a given camera id
     */
    D400CameraInterface() = delete;

    /**
     * Creates new instance that pulls data
     * from the camera with the given serial number
     */
    explicit D400CameraInterface(YAML::Node config);

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

    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudBasic() const override;

    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getMappedPointCloud() const override;

    virtual void disableAutoExposure() override;

    /**
     * increments the camera to the next available frame
     * all the get methods now pull data from the most recent frame
     */
    virtual bool loadNext() override;

    /**
     * increments to the next frame using loadNext
     * Returns the LegacyCameraInterface instance following the increment
     */
    virtual D400CameraInterface& operator++() override;

    virtual maav::vision::CameraPoseData getPoseData();

    const void* getRawDepth() const;
    const void* getRawColor() const;

    int getStreamWidth() const;
    int getStreamHeight() const;

    uint64_t getUTime() const;

private:
    bool enabled_;
    bool publish_pos_;
    std::string serial_;
    int width_;
    int height_;
    int fps_;

    const uint16_t* depth_image_;
    const uint16_t* color_image_;

    uint64_t utime_;

    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::frameset frames_;
    rs2::frame rgb_frame_;
    rs2::frame depth_frame_;
    rs2::frame pose_frame_;
    rs2_intrinsics depth_intrinsics_;
    rs2_extrinsics depth_to_color_;
    rs2_intrinsics color_intrinsics_;
    std::unique_ptr<rs2::align> align_object_;
    float scale_;

    rs2::sensor sensor_color_;
    rs2::sensor sensor_depth_;
};
}  // namespace maav::vision

#endif
