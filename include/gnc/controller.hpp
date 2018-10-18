#pragma once

#include <common/mavlink/offboard_control.hpp>
#include <gnc/measurements/waypoint.hpp>
#include <gnc/state.hpp>

namespace maav
{
namespace gnc
{
class Controller
{
   public:
	Controller();
	~Controller();

	// TODO: create target struct
	void set_target(const Waypoint& waypoint);

	// TODO: return actuator controls
	void add_state(const State& state);

	void run();

   private:
	maav::mavlink::OffboardControl offboard_control;
};

}  // namespace gnc
}  // namespace maav



/* Inteface to offboard control
 * ===============================================================
 * 
 * void set_attitude_target(const InnerLoopSetpoint& new_setpoint)
 * 
 * void set_zero_attitude();  //good for establishing control(set zero attitude and thrust)
 * 
 */