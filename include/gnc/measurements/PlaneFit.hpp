#pragma once

namespace maav {
namespace gnc {
namespace measurements {

/*
 * Stores the necessary info we need from a plane fitting message.
 */
struct PlaneFitMeasurement {
    double height;
    double vertical_speed;
    double roll;
    double pitch;
    uint64_t time;
};

}  // namespace measurements
}  // namespace gnc
}  // namespace maav
