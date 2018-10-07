#include <gnc/utils/zcm_conversion.hpp>

namespace maav
{
namespace gnc
{
State convert_state(const state_t& zcm_state)
{
	// Convert to BaseState
	State state(zcm_state.utime);
	Eigen::Vector3d av(zcm_state.angular_velocity[0], zcm_state.angular_velocity[1],
					   zcm_state.angular_velocity[2]);
	Eigen::Vector3d pos(zcm_state.position[0], zcm_state.position[1], zcm_state.position[2]);
	Eigen::Vector3d vel(zcm_state.velocity[0], zcm_state.velocity[1], zcm_state.velocity[2]);
	Eigen::Vector3d accel(zcm_state.acceleration[0], zcm_state.acceleration[1],
						  zcm_state.acceleration[2]);
	state.angular_velocity() = av;
	state.position() = pos;
	state.velocity() = vel;
	state.acceleration() = accel;
	state.attitude() =
		Sophus::SO3d(Eigen::Quaterniond(zcm_state.attitude[0], zcm_state.attitude[1],
										zcm_state.attitude[2], zcm_state.attitude[3]));

	return state;
}

Waypoint convert_waypoint(const waypoint_t& zcm_waypoint)
{
	Waypoint waypoint;
	waypoint.position =
		Eigen::Vector3d(zcm_waypoint.pose[0], zcm_waypoint.pose[1], zcm_waypoint.pose[2]);
	waypoint.velocity =
		Eigen::Vector3d(zcm_waypoint.rate[0], zcm_waypoint.rate[1], zcm_waypoint.rate[2]);
	waypoint.yaw = zcm_waypoint.pose[3];

	return waypoint;
}
}
}
