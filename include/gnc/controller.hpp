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

	void take_off();
	void yaw_right();
	void yaw_left();
	void roll_left();
	void roll_right();
	void pitch_forward();
	void pitch_back();
	void land();

   private:
	maav::mavlink::OffboardControl offboard_control;
};

}  // namespace gnc
}  // namespace maav



/* Inteface to offboard control
 * ==============================
 * 
 * void set_zero_attitude()
 * --Sets level attitude, zero angle rates, zero thrust
 * 
 * void set_thrust(const float thrust);
 * --sets normalized thrust value 0<=thrust<=1
 * 
 * void set_yaw_rate(const float yaw_rate);
 * void set_roll_rate(const float roll_rate);
 * void set_pitch_rate(const float pitch_rate);
 * --sets yaw/pitch/roll rate in rad/s
 * 
 * 
 */