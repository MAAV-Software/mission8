#include <common/mavlink/communication_port.hpp>

namespace maav
{
namespace mavlink
{
CommunicationPort::CommunicationPort()
{
	udp_socket = socket(AF_INET, SOCK_DGRAM, 0);

	local_address.sin_family = AF_INET;
	local_address.sin_addr.s_addr = INADDR_ANY;
	local_address.sin_port = htons(14540);

	bind(udp_socket, (struct sockaddr *)&local_address, sizeof(sockaddr));
	fcntl(udp_socket, F_SETFL, O_NONBLOCK | O_ASYNC);

	remote_address.sin_family = AF_INET;
	remote_address.sin_addr.s_addr = inet_addr("127.0.0.1");
	remote_address.sin_port = htons(14557);
}

CommunicationPort::~CommunicationPort() { close(udp_socket); }
bool CommunicationPort::read_message(mavlink_message_t &message)
{
	uint8_t buffer[BUFFER_SIZE];
	memset(buffer, 0, BUFFER_SIZE);
	ssize_t recsize = read(udp_socket, buffer, BUFFER_SIZE);

	mavlink_status_t status;
	bool message_received = false;
	// using status so it doesnt through an error (bullshit short term fix)
	if (status.buffer_overrun > 0) message_received = false;

	if (recsize > 0)
	{
		for (int i = 0; i < recsize; ++i)
		{
			message_received = mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status);
		}
	}

	return message_received;
}

bool CommunicationPort::write_message(const mavlink_message_t &message)
{
	uint8_t buffer[BUFFER_SIZE];
	mavlink_msg_to_send_buffer(buffer, &message);
	return sendto(udp_socket, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&remote_address,
				  sizeof(struct sockaddr_in));
}

}  // maav
}  // mavlink