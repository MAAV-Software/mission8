#include <gnc/kalman/updates/GlobalUpdate.hpp>

using maav::gnc::measurements::GlobalUpdateMeasurement;

namespace maav
{
namespace gnc
{
namespace kalman
{
GlobalUpdate::GlobalUpdate(YAML::Node config) : BaseUpdate(config["global_update"]) {}
GlobalUpdateMeasurement GlobalUpdate::predicted(const State& state)
{
    /*
    Here, we predict the position and attitude of our quadcopter
    */
    GlobalUpdateMeasurement predicted_measurement;
    predicted_measurement.attitude() = state.attitude();
    predicted_measurement.position() = state.position();
    return predicted_measurement;
}

GlobalUpdateMeasurement GlobalUpdate::measured(const measurements::Measurement& meas)
{
    return *(meas.global_update);
}

void GlobalUpdate::operator()(History::Snapshot& snapshot)
{
    // Check the validity of the lidar measurements
    if (snapshot.measurement.global_update)
    {
        correct(snapshot);
    }
}
}  // namespace kalman
}  // namespace gnc
}  // namespace maav