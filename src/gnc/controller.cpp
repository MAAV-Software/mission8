#include "gnc/controller.hpp"
#include <iostream>

namespace maav
{
namespace gnc
{
Controller::Controller() {}
Controller::~Controller() {}
void Controller::set_target(const Waypoint& waypoint) {}
void Controller::add_state(const State& state) {}
void Controller::run() {}
void Controller::set_control_params(const ctrl_params_t& _params) { control_params = _params; }
}  // namespace gnc
}  // namespace maav
