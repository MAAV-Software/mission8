#include <gnc/estimator.hpp>
#include <gnc/state/base_state.hpp>

using maav::gnc::measurements::MeasurementSet;

namespace maav
{
namespace gnc
{
using namespace kalman;

Estimator::Estimator(YAML::Node config)
    : empty_state(0), history(config["history"], config["state"]), prediction(config["prediction"])
{
}
const State& Estimator::add_measurement_set(MeasurementSet& meas)
{
    auto it_pair = history.add_measurement(meas);
    History::Iterator begin = it_pair.first;
    History::Iterator end = it_pair.second;

    if (begin == end)
    {
        return empty_state;
    }

    History::Iterator prev, next;
    next = prev = begin;
    next++;

    while (next != end)
    {
        prediction(prev, next);
        // TODO: Add updates
        prev++;
        next++;
    }

    return prev->state;
}
}  // namespace gnc
}  // namespace maav
