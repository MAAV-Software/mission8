#pragma once

#include <memory>

#include "gnc/measurements/GlobalUpdate.hpp"
#include "gnc/measurements/Imu.hpp"
#include "gnc/measurements/Lidar.hpp"
#include "gnc/measurements/PlaneFit.hpp"
#include "gnc/measurements/VisualOdometry.hpp"

namespace maav {
namespace gnc {
namespace measurements {

/**
 * The Measurement class holds a group of measurements that are meant to be
 * together (so, they all happened at the same time).
 */
class Measurement {
    std::shared_ptr<ImuMeasurement> imu;
    std::shared_ptr<LidarMeasurement> lidar;
    std::shared_ptr<PlaneFitMeasurement> plane_fit;
    std::shared_ptr<VisualOdometryMeasurement> visual_odometry;
    std::shared_ptr<GlobalUpdateMeasurement> global_update;
};

/*
 * The MeasurementSet class is the same as the Measurement class.
 * The reason we give it a different name is that it is used for a different
 * purpose. Where the Measurement class is meant to represent all the
 * measurements that we have from a particular moment in time, the
 * MeasurementSet class is used to hold a collection of measurements that may
 * not have happened at the same moment, mostly for convenience so we don't
 * have to split up measurements into many arguments.
 */
typedef Measurement MeasurementSet;

}  // namespace measurements
}  // namespace gnc
}  // namespace maav
