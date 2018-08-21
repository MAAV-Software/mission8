#pragma once

#include <cstdlib>
#include <list>

#include "gnc/kalman/kalman_state.hpp"
#include "gnc/measurements/Measurement.hpp"

namespace maav {
namespace gnc {
namespace kalman {

/**
 * Time series of sensor measurements and their corresponding estimated
 * states.
 */
class History {
   public:
    explicit History(size_t size);

   public:
    /**
     * Snapshots store all information about a state at a specific time.
     */
    struct Snapshot {
        Snapshot(const KalmanState& state_,
                 const measurements::Measurement& meas_)
            : state(state_), measurement(meas_) {}

        KalmanState state;
        measurements::Measurement measurement;

        uint64_t get_time() const { return state.time_usec(); }
    };

    using Iterator = std::list<History::Snapshot>::iterator;

    /**
     * Takes a MeasurementSet of all new measurements. Adds all of those
     * into the history, and returns a pair of iterators to the first
     * element that was changed and the end.
     *
     * IMU must be populated
     */
    std::pair<Iterator, Iterator> add_measurement(
        measurements::MeasurementSet& measurements);

    size_t size();

   private:
    void resize(Iterator last_modified);

    /***
     * Finds a snapshot within tolerance of the given time or creates a new
     * snapshot with an interpolated IMU measurement.
     * @param time
     * @param tolerance
     * @return
     */
    Iterator find_snapshot(uint64_t time, uint64_t tolerance);

    measurements::ImuMeasurement interpolate_imu(
        const measurements::ImuMeasurement& prev,
        const measurements::ImuMeasurement& next, uint64_t interp_time) const;

    Iterator set_last_modified(const Iterator last_modified,
                               const Iterator modified) const;

   private:
    size_t _size;

    std::list<History::Snapshot> _history;
};

}  // namespace kalman
}  // namespace gnc
}  // namespace maav