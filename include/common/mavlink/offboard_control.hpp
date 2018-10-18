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

struct InnerLoopSetpoint
{
	Eigen::Quaternion<float> q{1, 0, 0, 0};
	float thrust = 0;			// 0 <= thrust <= 1
	float roll_rate = 0;		// rad/s
	float pitch_rate = 0;		// rad/s
	float yaw_rate = 0;			// rad/s
};

class OffboardControl
{
   public:
	OffboardControl();
	~OffboardControl();

	void set_attitude_target(const InnerLoopSetpoint& new_setpoint);
	void set_zero_attitude();  //good for establishing control

   private:
	bool read_message();
	void read_thread();
	void write_message(const mavlink_message_t& message);
	void check_offboard_control();
	void hold_zero_attitude(const uint64_t seconds);

	void ping(const uint64_t boot_timestamp);
	void send_heartbeat();
	void activate_offboard_control();
	void arm();

	Messages current_messages_in;
	std::thread read_tid;
	std::thread write_tid;
	CommunicationPort com_port;

	bool offboard_control_active;

	// These are defined so that they are not magic numbers in the code
	const uint8_t system_id = 1;	 // system we are connecting should always be 1 (only system)
	const uint8_t autopilot_id = 1;  // componenet we are controlling should always be 1 (autopilot)
	const uint8_t companion_id = 2;
	union px4_custom_mode custom_mode;

	friend void read_thread_start(OffboardControl* offboard_control);
};

}  // maav
}  // gnc
