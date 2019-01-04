#include <gnc/State.hpp>
#include <gnc/localizer.hpp>

using cv::Mat;

namespace maav
{
namespace gnc
{
Localizer::Localizer(SlamInitializer& slam_init)
    : slam(slam_init.vocabulary_file, slam_init.config_file, slam_init.sensor, slam_init.zcm_url,
          slam_init.send_images)
{
}

Localizer::~Localizer() { slam.Shutdown(); }
void Localizer::addImage(const Mat& color, const Mat& depth, uint64_t timestamp)
{
    constexpr double MICROSECONDS_PER_SECOND = 1000000;
    double seconds = static_cast<double>(timestamp) / MICROSECONDS_PER_SECOND;
    current_pose = slam.TrackRGBD(color, depth, seconds);
}

void Localizer::addImu(/*const measurements::ImuMeasurement& imu*/)
{
    // TODO: IMU frontend + On Manifold Preintegration + Bias
}

cv::Mat Localizer::getPose() { return current_pose; }
map_t Localizer::getMap()
{
    assert(false);
    map_t map;
    return map;
}

}  // namespace gnc
}  // namespace maav