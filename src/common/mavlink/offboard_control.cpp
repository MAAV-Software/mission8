#include <common/mavlink/offboard_control.hpp>

#include <unistd.h>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>

using namespace std::chrono;

namespace maav
{
namespace mavlink
{
std::atomic<bool> KILL{false};

// Creates thread and passing read_messages loop into thread
OffboardControl::OffboardControl() { read_tid = std::thread(read_thread_start, this); }
OffboardControl::~OffboardControl()
{
	KILL = true;
	read_tid.join();
}

void OffboardControl::read_thread()
{
	while (!KILL)
	{
		read_message();

		std::this_thread::sleep_for(10ms);  // sample at 100 Hz
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
				print_new_messages();
				break;
			case MAVLINK_MSG_ID_SYSTEM_TIME:
				mavlink_msg_system_time_decode(&message, &(current_messages_in.system_time));
				ping(current_messages_in.system_time.time_boot_ms * 1e3);
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

void OffboardControl::print_new_messages()
{
	std::cout << "custom mode: ";
	std::cout << current_messages_in.heartbeat.custom_mode << '\n';
	std::cout << "type: ";
	std::cout << (int)current_messages_in.heartbeat.type << '\n';
	std::cout << "autopilot: ";
	std::cout << (int)current_messages_in.heartbeat.autopilot << '\n';
	std::cout << "base_mode: ";
	std::cout << (int)current_messages_in.heartbeat.base_mode << '\n';
	std::cout << "system status: ";
	std::cout << (int)current_messages_in.heartbeat.system_status << '\n';
	std::cout << "mavlink version: ";
	std::cout << (int)current_messages_in.heartbeat.mavlink_version << '\n';
}

void OffboardControl::ping(const uint64_t boot_timestamp)
{
	mavlink_ping_t ping_message;
	ping_message.target_system = 1;
	ping_message.target_component = 1;
	ping_message.seq = 1234;
	ping_message.time_usec = boot_timestamp;

	mavlink_message_t message;
	mavlink_msg_ping_encode(1, 1, &message, &ping_message);

	write_message(message);
}

// helper for starting read thread
void read_thread_start(OffboardControl* OffboardControl) { OffboardControl->read_thread(); }
}  // maav
}  // gnc
