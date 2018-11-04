#pragma once

#include "gnc/measurements/Measurement.hpp"
#include "gnc/state.hpp"

namespace maav
{
namespace gnc
{
class Estimator
{
   public:
    Estimator();

    const State& add_measurement_set(const measurements::MeasurementSet& meas);

   private:
    State state;
};

}  // namespace gnc
}  // namespace maav
