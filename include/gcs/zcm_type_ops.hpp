#ifndef MAAV_GCS_ZCM_TYPE_OPS_HPP
#define MAAV_GCS_ZCM_TYPE_OPS_HPP

#include "Messages.hpp"

#include <algorithm>
#include <iterator>

inline bool operator==(const pid_gains_t& a, const pid_gains_t& b)
{
	return a.p == b.p and a.i == b.i and a.d == b.d;
}

inline bool operator!=(const pid_gains_t& a, const pid_gains_t& b) { return not(a == b); }
inline bool operator==(const ctrl_params_t& a, const ctrl_params_t& b)
{
	using namespace std;
	return equal(begin(a.value), end(a.value), begin(b.value)) and
		   equal(begin(a.rate), end(a.rate), begin(b.rate));
}

inline bool operator!=(const ctrl_params_t& a, const ctrl_params_t& b) { return not(a == b); }
inline bool operator==(const waypoint_t& a, const waypoint_t& b)
{
	return a.mode == b.mode and a.pmode == b.pmode and
		   std::equal(std::begin(a.pose), std::end(a.pose), std::begin(b.pose)) and
		   std::equal(std::begin(a.rate), std::end(a.rate), std::begin(b.rate));
}

inline bool operator!=(const waypoint_t& a, const waypoint_t& b) { return not(a == b); }
inline bool operator==(const nav_runstate_t& a, const nav_runstate_t& b)
{
	return a.running_mission == b.running_mission;
}

inline bool operator!=(const nav_runstate_t& a, const nav_runstate_t& b) { return not(a == b); }
inline bool operator==(const camera_disc_t& a, const camera_disc_t& b)
{
	return a.numCameras == b.numCameras;
}

inline bool operator!=(const camera_disc_t& a, const camera_disc_t& b) { return not(a == b); }
inline bool operator==(const dji_t& a, const dji_t& b)
{
	return a.utime == b.utime && a.roll == b.roll && a.pitch == b.pitch && a.yawRate == b.yawRate &&
		   a.thrust == b.thrust;
}

inline bool operator!=(const dji_t& a, const dji_t& b) { return not(a == b); }
inline bool operator==(const idle_t& a, const idle_t& b)
{
	return a.idle == b.idle and a.utime == b.utime;
}

inline bool operator!=(const idle_t& a, const idle_t& b) { return not(a == b); }
#endif
