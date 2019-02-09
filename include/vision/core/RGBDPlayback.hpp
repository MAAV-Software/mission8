#ifndef RGBDGETTER_HPP
#define RGBDGETTER_HPP

#include <cstdint>
#include <fstream>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdio>
#include <librealsense/rs.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>

namespace maav::vision
{
/**
 * An iterator-like class for the purposes of extracting
 * RGBD data from the filesystem and providing it in various forms
 */
class RGBDPlayback
{
public:
    /**
     * Creates a new RGBDPlayback with the given numbered directory
     *
     * _num : the number of the directory to pull RGBD data from
     * path: path to directory with frames
     */
    explicit RGBDPlayback(int num, const std::string& path);

    /**
             * Creates a new RGBDPlayback with the given numbered directory
             * and default path
             *
             * _num : the number of the directory to pull RGBD data from
             */
    RGBDPlayback(int num);

    /**
     * an RGBDPlayback cannot be created without a specified directory
     */
    RGBDPlayback() = delete;

    /**
     * pulls the RGB data for the frame at the present counter position
     * and provides it in a cv::Mat
     *
     * img : a cv::Mat reference to hold the returned RGB data
     */
    cv::Mat getRGB();

    /**
     * pulls the Depth data for the frame at the present counter position
     * and provides it in a cv::Mat
     *
     * img : a cv::Mat reference to hold the returned depth data
     */
    cv::Mat getDepth();

    /**
     * pulls the combined RGB & Depth (RGBD) data for the frame at the
     * present counter position and provides it in a cv::Mat
     *
     * img : a cv::Mat reference to hold the returned RGBD data
     */
    cv::Mat getCombined();

    /**
     * pulls the Point Cloud data for the frame at the present counter
     * position and provides it in a pcl::PointCloud
     * -- gives the XYZ data for each point
     *
     * cloud : a pcl::PointCloud reference to hold the returned data
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();

    /**
     * increments the counter to the next frame
     * all the get methods now pull from the following frame
     * the timestamp is updated to the time of the next frame
     */
    RGBDPlayback& operator++();

private:
    unsigned counter_ = 0;
    int64_t timestamp_ = 0;
    std::ifstream timestamps_;
    std::string dir_num_;
    std::string file_path_;
    void openMatFromDirectory(const std::string&, cv::Mat&);
};
}
// RGBDPlayback

#endif
