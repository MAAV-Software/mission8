#pragma once

#include <thread>

#include <mavlink/v2.0/common/mavlink.h>
#include <mavlink/v2.0/px4_custom_mode.h>
#include <Eigen/Eigen>
#include "communication_port.hpp"

namespace maav
{
namespace mavlink
{
struct Messages
{
	mavlink_heartbeat_t heartbeat;
	mavlink_system_time_t system_time;
	mavlink_ping_t ping;
};

struct Setpoint
{
	Eigen::Quaternion<float> q{1, 0, 0, 0};
	float thrust = 0;	// 0 <= thrust <= 1
	float roll_rate = 0;		// rad/s
	float pitch_rate = 0;	// rad/s
	float yaw_rate = 0;		// rad/s
};

class OffboardControl
{
   public:
	OffboardControl();
	~OffboardControl();

	//
	// PIXHAWK MIGHT NOT ACCEPT THESE COMMANDS INDIVIDUALLY
	//
	void set_zero_attitude();
	void set_thrust(const float thrust);
	void set_yaw_rate(const float yaw_rate);
	void set_roll_rate(const float roll_rate);
	void set_pitch_rate(const float pitch_rate);
	void set_attitude(const Eigen::Quaternion<float>& q);
	void zero_rates();

	void update_setpoint(){

	}

   private:
	void read_message();
	void read_thread();
	void write_message(const mavlink_message_t& message);
	void check_offboard_control();

	void ping(const uint64_t boot_timestamp);
	void send_heartbeat();
	void activate_offboard_control();

	void set_attitude_target(const Setpoint& new_setpoint, const uint8_t type_mask);

	Messages current_messages_in;
	std::thread read_tid;
	std::thread write_tid;
	CommunicationPort com_port;

	bool offboard_control_active;

	// These are defined so that they are not magic numbers in the code
	const uint8_t system_id = 1;	 // system we are connecting should always be 1 (only system)
	const uint8_t autopilot_id = 1;  // componenet we are controlling should always be 1 (autopilot)
	const uint8_t companion_id =
		0;  // this is the OffboardController id i think (not really sure abou this one)
	union px4_custom_mode custom_mode;

	Setpoint current_setpoint;

	// Define type masks for set_attitude_target
	static constexpr uint8_t SET_ALL = 0b00000000;
	static constexpr uint8_t SET_THRUST = 0b10000111;
	static constexpr uint8_t SET_ATTITUDE = 0b01000111;
	static constexpr uint8_t SET_YAW_RATE = 0b11000011;
	static constexpr uint8_t SET_PITCH_RATE = 0b11000101;
	static constexpr uint8_t SET_ROLL_RATE = 0b11000110;

	friend void read_thread_start(OffboardControl* offboard_control);
};

}  // maav
}  // gnc
