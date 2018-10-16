#include <common/mavlink/offboard_control.hpp>

#include <unistd.h>
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <mutex>

using namespace std::chrono;
using std::this_thread::sleep_for;
using std::thread;
using std::cout;

namespace maav
{
namespace mavlink
{
std::atomic<bool> KILL{false};

// helper for starting read thread
void read_thread_start(OffboardControl* OffboardControl) { OffboardControl->read_thread(); }

// Creates thread and passing read_messages loop into thread
OffboardControl::OffboardControl()
{
	// set custom mode to offboard control (for comparisons)
	custom_mode.data = 0;
	custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;

	// Start read thread and heartbeat (TODO: datalink like Qgroundcontrol)
	read_tid = thread(read_thread_start, this);

	// Establish Offboard Control
	// must establish heartbeat to pixhawk and set attitude
	// or pixhawk will enter failsafe (disable offboard and RTL)
	offboard_control_active = false;
	uint64_t timeout_start = time(NULL);
	while (!offboard_control_active)
	{
		set_zero_attitude();
		sleep_for(100ms);

		activate_offboard_control();

		if (current_messages_in.heartbeat.custom_mode == custom_mode.data)
		{
			offboard_control_active = true;
			break;
		}

		// Timeout
		if ((time(NULL) - timeout_start) > 10)
		{
			cout << "Timeout establishing offboard control of pixhawk\n\n";
			assert(false);
		}
	}

	cout << "Established offboard control on pixhawk\n\n";
}

OffboardControl::~OffboardControl()
{
	KILL = true;
	read_tid.join();
}

void OffboardControl::read_thread()
{
	while (!KILL)
	{
		read_message();  // read incomming messages

		send_heartbeat();  // maintain link with pixhawk, if link lost pixhawk -> failsafe
		// This heartbeat is probably too fast, consider lowering rate (what is the minimum?)

		sleep_for(10ms);  // sample at 100 Hz
	}
}

void OffboardControl::read_message()
{
	mavlink_message_t message;
	bool message_received = false;

	message_received = com_port.read_message(message);

	if (message_received)
	{
		switch (message.msgid)
		{
			case MAVLINK_MSG_ID_HEARTBEAT:
				mavlink_msg_heartbeat_decode(&message, &(current_messages_in.heartbeat));
				check_offboard_control();  // check each heartbeat
				// TODO: handle loss of heartbeat
				break;
			case MAVLINK_MSG_ID_SYSTEM_TIME:
				mavlink_msg_system_time_decode(&message, &(current_messages_in.system_time));
				ping(current_messages_in.system_time.time_boot_ms * 1e3);  // convert to us
				break;
			default:
				break;
		}
	}
}

void OffboardControl::write_message(const mavlink_message_t& message)
{
	// Currently doesnt provide much additional functionality but it
	// abstracts away the com port write
	com_port.write_message(message);
}

void OffboardControl::check_offboard_control()
{
	if (current_messages_in.heartbeat.custom_mode != custom_mode.data && offboard_control_active)
	{
		offboard_control_active = false;
		cout << "Lost offboard control on pixhawk\n\n";
	}
}

void OffboardControl::ping(const uint64_t boot_timestamp)
{
	mavlink_ping_t ping_message;
	ping_message.target_system = system_id;
	ping_message.target_component = autopilot_id;
	ping_message.seq = 1234;
	ping_message.time_usec = boot_timestamp;

	mavlink_message_t message;
	mavlink_msg_ping_encode(system_id, companion_id, &message, &ping_message);

	write_message(message);
}

void OffboardControl::send_heartbeat()
{
	mavlink_heartbeat_t heartbeat;
	heartbeat.type = MAV_TYPE_GCS;
	heartbeat.autopilot = MAV_AUTOPILOT_INVALID;  // identify as GCS/offboard control
	heartbeat.base_mode = 0;					  // no relavent mode for gcs, set to 0
	heartbeat.system_status = MAV_STATE_ACTIVE;
	heartbeat.custom_mode = 0;

	mavlink_message_t message;
	mavlink_msg_heartbeat_encode(system_id, companion_id, &message, &heartbeat);
	write_message(message);
}

void OffboardControl::activate_offboard_control()
{
	// This command will fail if the pixhawk is not
	// receiving offboard position/attitude commands
	// at a rate >2 Hz

	mavlink_command_long_t command;
	command.command = MAV_CMD_NAV_GUIDED_ENABLE;
	command.target_system = system_id;
	command.target_component = autopilot_id;
	command.confirmation = true;  // idk what this is doing
	command.param1 = (float)1;	// >0.5 activate, <0.5 deactivate

	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &command);
	write_message(message);
}

void OffboardControl::set_attitude_target(const Eigen::Quaternion<float>& q, const float thrust,
										  const float roll_rate, const float pitch_rate,
										  const float yaw_rate, const uint8_t type_mask)
{
	assert(thrust >= 0 && thrust <= 1);

	mavlink_set_attitude_target_t setpoint;
	setpoint.time_boot_ms = current_messages_in.system_time.time_boot_ms;
	setpoint.target_system = system_id;
	setpoint.target_component = autopilot_id;
	setpoint.type_mask = type_mask;

	setpoint.q[0] = q.w();
	setpoint.q[1] = q.x();
	setpoint.q[2] = q.y();
	setpoint.q[3] = q.z();
	setpoint.body_roll_rate = roll_rate;
	setpoint.body_pitch_rate = pitch_rate;
	setpoint.body_yaw_rate = yaw_rate;
	setpoint.thrust = thrust;

	mavlink_message_t message;
	mavlink_msg_set_attitude_target_encode(system_id, companion_id, &message, &setpoint);
	write_message(message);
}

void OffboardControl::set_zero_attitude()
{
	set_attitude_target(Eigen::Quaternion<float>(1., 0., 0., 0.), 0, 0, 0, 0, SET_ALL);
}

//Requires 0<= thrust <= 1
void OffboardControl::set_thrust(const float thrust){
	assert(thrust >= 0 && thrust <= 1);
	set_attitude_target(Eigen::Quaternion<float>(1., 0., 0., 0.), thrust, 0., 0., 0., SET_THRUST);
}

void OffboardControl::set_yaw_rate(const float yaw_rate){
	set_attitude_target(Eigen::Quaternion<float>(1.,0.,0.,0.), 0., 0., 0., yaw_rate, SET_YAW_RATE);
}

void OffboardControl::set_pitch_rate(const float pitch_rate){
	set_attitude_target(Eigen::Quaternion<float>(1.,0.,0.,0.), 0., 0., pitch_rate, 0., SET_PITCH_RATE);
}

void OffboardControl::set_roll_rate(const float roll_rate){
	set_attitude_target(Eigen::Quaternion<float>(1.,0.,0.,0.), 0., roll_rate, 0., 0., SET_ROLL_RATE);
}
}  // maav
}  // gnc
