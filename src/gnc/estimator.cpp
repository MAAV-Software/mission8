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
      global_update_(config["updates"]),
      enable_lidar(config["en_lidar"].as<bool>()),
      enable_planefit(config["en_planefit"].as<bool>()),
      enable_global(config["en_global"].as<bool>())
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
        // lidar_update_(*next);
        planefit_update_(*next);

        if (enable_lidar) lidar_update_(*next);
        if (enable_planefit) planefit_update_(*next);
        if (enable_global) global_update_(*next);

        prev++;
        next++;
    }
    return prev->state;
}

}  // namespace gnc
}  // namespace maav
