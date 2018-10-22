#pragma once

#include <common/mavlink/offboard_control.hpp>
#include <gnc/measurements/waypoint.hpp>
#include <gnc/state.hpp>
#include <common/messages/ctrl_params_t.hpp>

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

	void set_control_params(const ctrl_params_t& _params);

   private:
   	ctrl_params_t control_params;

};

}  // namespace gnc
}  // namespace maav