#pragma once

#include "gnc/state.hpp"

namespace maav
{
namespace gnc
{
class Controller
{
   public:
	Controller();

	// TODO: create target struct
	void set_target(void);

	// TODO: return actuator controls
	void add_state(/*const State& state*/);
};

}  // namespace gnc
}  // namespace maav
