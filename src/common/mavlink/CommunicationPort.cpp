#include <common/mavlink/CommunicationPort.hpp>

using std::string;

namespace maav
{
namespace mavlink
{
CommunicationPort::CommunicationPort(CommunicationType type, const string &port_path)
    : com_type(type)
{
    switch (com_type)
    {
        case CommunicationType::UDP:
            udp_socket = socket(AF_INET, SOCK_DGRAM, 0);

            local_address.sin_family = AF_INET;
            local_address.sin_addr.s_addr = INADDR_ANY;
            local_address.sin_port = htons(14540);

            bind(udp_socket, (struct sockaddr *)&local_address, sizeof(sockaddr));
            fcntl(udp_socket, F_SETFL, O_NONBLOCK | O_ASYNC);

            remote_address.sin_family = AF_INET;
            remote_address.sin_addr.s_addr = inet_addr("127.0.0.1");
            remote_address.sin_port = htons(14557);
            break;
        case CommunicationType::UART:
            uart_port.connect(port_path.c_str());
            break;
    }
}

CommunicationPort::~CommunicationPort()
{
    switch (com_type)
    {
        case CommunicationType::UDP:
            close(udp_socket);
            break;
        case CommunicationType::UART:
            uart_port.disconnect();
            break;
    }
}

bool CommunicationPort::read_message(mavlink_message_t &message)
{
    char buffer[BUFFER_SIZE];
    memset(buffer, 0, BUFFER_SIZE);

    ssize_t recsize = 0;

    switch (com_type)
    {
        case CommunicationType::UDP:
            recsize = read(udp_socket, buffer, BUFFER_SIZE);
            break;
        case CommunicationType::UART:
            recsize = uart_port.receive(buffer, BUFFER_SIZE);
            break;
    }

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
void CommunicationPort::write_message(const mavlink_message_t &message)
{
    char buffer[BUFFER_SIZE];
    mavlink_msg_to_send_buffer((uint8_t *)buffer, &message);

    switch (com_type)
    {
        case CommunicationType::UDP:
            sendto(udp_socket, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&remote_address,
                sizeof(struct sockaddr_in));
            break;
        case CommunicationType::UART:
            uart_port.send((char *)buffer, BUFFER_SIZE);
            break;
    }
}

}  // namespace mavlink
}  // namespace maav