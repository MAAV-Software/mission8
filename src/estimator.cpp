#include "gnc/estimator.hpp"
#include "gnc/state/base_state.hpp"

using maav::gnc::measurements::MeasurementSet;

namespace maav {
namespace gnc {

Estimator::Estimator() : state(0) {}

const State& Estimator::add_measurement_set(/*const MeasurementSet& meas*/) {
    return state;
}

}  // namespace gnc
}  // namespace maav