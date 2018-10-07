#pragma once

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

	// TODO: create target struct
	void set_target(const Waypoint& waypoint);

	// TODO: return actuator controls
	void add_state(const State& state);
};

}  // namespace gnc
}  // namespace maav
