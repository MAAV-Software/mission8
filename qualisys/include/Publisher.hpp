#ifndef MAAV_QUALISYS_PUBLISHER
#define MAAV_QUALISYS_PUBLISHER

#include <string>
#include <zmq.hpp>

namespace qualisys
{
/**
 * @brief A Publisher handles sending a Msg type over TCP using the given IP
 *
 * @tparam Msg A message type with the fucntions `getEncodedSize` and `encode`. Any ZCM message will
 * work.
 */
template <class Msg>
class Publisher
{
public:
    /**
     * @brief Construct a new Publisher object and connect
     *
     * @details The IP can simply be "*" in which case it is sent to all all listening connections.
     * You can also choose any free port.
     */
    Publisher(const std::string& ip, int port) : sock_{ctx_, ZMQ_PUB} { connect(ip, port); }

    /**
     * @brief Construct a new Publisher object without a connection
     */
    Publisher() : sock_{ctx_, ZMQ_PUB} {}

    /**
     * @brief Destroy the Publisher object disconnecting the socket and closing the context.
     */
    ~Publisher()
    {
        sock_.close();
        ctx_.close();
    }

    /**
     * @brief Connect the socket
     */
    void connect(const std::string& ip, int port)
    {
        std::string transport = "tcp://" + ip + ":" + std::to_string(port);
        sock_.setsockopt(ZMQ_SNDTIMEO, 0);  // Do not wait for receiver
        sock_.bind(transport);
    }

    /**
     * @brief Send this message.
     */
    void send(const Msg& msg)
    {
        uint32_t size = msg.getEncodedSize();
        zmq::message_t tx{size};
        void* buf = malloc(size);
        msg.encode(buf, 0, size);
        memcpy(tx.data(), buf, size);
        sock_.send(tx);
        free(buf);
    }

private:
    zmq::context_t ctx_;
    zmq::socket_t sock_;
};
}  // namespace qualisys
#endif