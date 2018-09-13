#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "gnc/measurements/Imu.hpp"
#include "gnc/slam/System.h"

namespace maav {
namespace gnc {

struct SlamInitializer {
    std::string vocabulary_file;
    std::string config_file;
    slam::System::eSensor sensor;
    bool use_viewer = true;
};

class Localizer {
   public:
    Localizer(SlamInitializer& slam_init);

    ~Localizer();

    void add_image(const cv::Mat& color, const cv::Mat& depth,
                   uint64_t timestamp);

    // TODO: Add imu to graph
    void add_imu(/*const measurements::ImuMeasurement& imu*/);

   private:
    slam::System slam;
};

}  // namespace gnc
}  // namespace maav
