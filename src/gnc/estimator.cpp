#include <cmath>

#include <gnc/State.hpp>
#include <gnc/estimator.hpp>

using maav::gnc::measurements::MeasurementSet;

namespace maav
{
namespace gnc
{
using namespace kalman;

Estimator::Estimator(YAML::Node config)
    : empty_state_(0),
      history_(config["history"], config["state"]),
      prediction_(config["prediction"]),
      lidar_update_(config["updates"]),
      planefit_update_(config["updates"]),
      global_update_(config["updates"])
{
}

const State& Estimator::add_measurement_set(const MeasurementSet& meas)
{
    auto it_pair = history_.add_measurement(meas);
    History::Iterator begin = it_pair.first;
    History::Iterator end = it_pair.second;

    if (begin == end)
    {
        return empty_state_;
    }

    History::Iterator prev, next;
    next = prev = begin;
    next++;

    while (next != end)
    {
        prediction_(prev, next);
        lidar_update_(*next);
        planefit_update_(*next);
        global_update_(*next);

        prev++;
        next++;
    }

    std::cout << "Attitude: " << prev->state.attitude().unit_quaternion().w() << ' '
              << prev->state.attitude().unit_quaternion().x() << ' '
              << prev->state.attitude().unit_quaternion().y() << ' '
              << prev->state.attitude().unit_quaternion().z() << '\n';
    std::cout << "Position: " << prev->state.position().transpose() << '\n';
    std::cout << "Velocity: " << prev->state.velocity().transpose() << std::endl;
    std::cout << "Variance:\n" << prev->state.covariance().diagonal().transpose() << std::endl;
    return prev->state;
}

}  // namespace gnc
}  // namespace maav
