#include "gnc.hpp"

using namespace cv;

namespace maav {
namespace gnc {

Localizer::Localizer(SlamInitializer& slam_init)
    : slam(slam_init.vocabulary_file, slam_init.config_file, slam_init.sensor,
           slam_init.use_viewer) {}

Localizer::~Localizer() { slam.Shutdown(); }

void Localizer::dump_image(const Mat& color, const Mat& depth,
                           uint64_t timestamp) {
    constexpr double MICROSECONDS_PER_SECOND = 1000000;
    double seconds = static_cast<double>(timestamp) / MICROSECONDS_PER_SECOND;
    slam.TrackRGBD(color, depth, seconds);
}

}  // namespace gnc
}  // namespace maav