#pragma once

#include <unistd.h>
#include <cstdint>
#include <iostream>
#include <string>

#include <sys/time.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <common/utils/SerialTTY.hpp>

namespace maav
{
namespace mavlink
{
enum class CommunicationType
{
	UDP,
	UART
};

constexpr size_t BUFFER_SIZE = 300;

class CommunicationPort
{
   public:
	CommunicationPort(CommunicationType type, const std::string &port_path);
	~CommunicationPort();
	bool read_message(mavlink_message_t &message);
	void write_message(const mavlink_message_t &message);

   private:
	CommunicationType com_type;

	struct sockaddr_in local_address;
	struct sockaddr_in remote_address;
	socklen_t addrlength;
	int udp_socket;

	// UART
	SerialTTY uart_port;
};

}  // maav
}  // mavlink
