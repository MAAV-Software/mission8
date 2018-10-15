#pragma once

#include <unistd.h>
#include <cstdint>
#include <iostream>

#include <sys/time.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <mavlink/v2.0/common/mavlink.h>

namespace maav
{
namespace mavlink
{
constexpr size_t BUFFER_SIZE = 300;

class CommunicationPort
{
   public:
	CommunicationPort();
	~CommunicationPort();
	bool read_message(mavlink_message_t &message);
	bool write_message(const mavlink_message_t &message);

   private:
	struct sockaddr_in local_address;
	struct sockaddr_in remote_address;
	socklen_t addrlength;
	int udp_socket;
};

}  // maav
}  // mavlink
