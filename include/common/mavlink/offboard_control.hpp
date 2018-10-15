#pragma once

#include <mavlink/v2.0/common/mavlink.h>
#include <thread>
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

class OffboardControl
{
   public:
	OffboardControl();
	~OffboardControl();

	void read_message();
	void read_thread();
	void write_message(const mavlink_message_t& message);
	void print_new_messages();

	void ping(const uint64_t boot_timestamp);

   private:
	Messages current_messages_in;
	std::thread read_tid;
	std::thread write_tid;
	CommunicationPort com_port;
};

void read_thread_start(OffboardControl* offboard_control);

}  // maav
}  // gnc
