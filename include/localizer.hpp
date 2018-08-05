#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "measurements.hpp"
#include "slam/System.h"

namespace maav {
namespace gnc {

struct SlamInitializer {
    std::string vocabulary_file;
    std::string config_file;
    ORB_SLAM2::System::eSensor sensor;
    bool use_viewer = true;
};

class Localizer {
   public:
    Localizer(SlamInitializer& slam_init);

    ~Localizer();

    void add_image(const cv::Mat& color, const cv::Mat& depth,
                   uint64_t timestamp);

    void add_imu(const measurements::ImuMeasurement& imu);

   private:
    ORB_SLAM2::System slam;
};

}  // namespace gnc
}  // namespace maav
