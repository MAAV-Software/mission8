#pragma once

#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>
#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
namespace sensors
{
// Forward declare private data class.
class PlaneFitSensorPrivate;

/// \addtogroup gazebo_sensors
/// \{

/// \class PlaneFitSensor PlaneFitSensor.hh sensors/sensors.hh
/// \brief PlaneFitSensor to interact with PlaneFitSensors
class GZ_SENSORS_VISIBLE PlaneFitSensor : public Sensor
{
    /// \brief Constructor.
public:
    PlaneFitSensor();

    /// \brief Destructor.
public:
    virtual ~PlaneFitSensor();

    // Documentation inherited
public:
    virtual void Load(const std::string& _worldName, sdf::ElementPtr _sdf);

    // Documentation inherited
public:
    virtual void Load(const std::string& _worldName);

    // Documentation inherited
public:
    virtual void Init();

    // Documentation inherited
protected:
    virtual bool UpdateImpl(const bool _force);

    // Documentation inherited
public:
    virtual void Fini();

    /// \internal
    /// \brief Private data pointer.
private:
    std::unique_ptr<PlaneFitSensorPrivate> dataPtr;
};

/// \}
}
}

using namespace gazebo;
using namespace sensors;
GZ_REGISTER_STATIC_SENSOR("plane_fit", PlaneFitSensor)
