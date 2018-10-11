#include <Eigen/Eigen>
#include <common/messages/state_t.hpp>
#include <common/messages/waypoint_t.hpp>
#include <gnc/measurements/waypoint.hpp>
#include <gnc/state.hpp>

namespace maav
{
namespace gnc
{
state_t convert_state(const State& state);
State convert_state(const state_t& state);
Waypoint convert_waypoint(const waypoint_t& zcm_waypoint);
}
}
