#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include <gnc/slam/System.h>
#include <common/messages/map_t.hpp>
#include <gnc/measurements/ImuMeasurement.hpp>

namespace maav
{
namespace gnc
{
struct SlamInitializer
{
    std::string vocabulary_file;
    std::string config_file;
    slam::System::eSensor sensor;
    bool use_viewer = true;
};

class Localizer
{
public:
    Localizer(SlamInitializer& slam_init);

    ~Localizer();

    void addImage(const cv::Mat& color, const cv::Mat& depth, uint64_t timestamp);

    // TODO: Add imu to graph
    void addImu(/*const measurements::ImuMeasurement& imu*/);

    // TODO: return a map
    map_t getMap();

    cv::Mat getPose();

private:
    slam::System slam;
    cv::Mat current_pose;
};

}  // namespace gnc
}  // namespace maav
