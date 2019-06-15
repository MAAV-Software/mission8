#ifndef MAAV_QUALISYS_SUBSCRIBER
#define MAAV_QUALISYS_SUBSCRIBER

#include <string>
#include <zmq.hpp>

namespace qualisys
{
/**
 * @brief A Subscriber listens to messages sent by a Publisher and constructs
 * the necessary ZCM message type
 *
 * @tparam Msg A message type with the fucntions `getEncodedSize` and `encode`. Any ZCM message will
 * work.
 */
template <class Msg>
class Subscriber
{
public:
    /**
     * @brief Construct a new Subscriber object and connect
     */
    Subscriber(const std::string& ip, int port) : sock_{ctx_, ZMQ_SUB} { connect(ip, port); }

    /**
     * @brief Construct a new Subscriber object without a connection
     *
     */
    Subscriber() : sock_{ctx_, ZMQ_SUB} {}

    /**
     * @brief Destroy the Subscriber object
     */
    ~Subscriber()
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
        sock_.connect(transport);
        sock_.setsockopt(ZMQ_SUBSCRIBE, "", 0);  // listen to eveything
    }

    /**
     * @brief Receive a message. This call blocks until a message is received.
     */
    Msg recv()
    {
        zmq::message_t rx;
        sock_.recv(&rx);
        Msg msg;
        msg.decode(rx.data(), 0, rx.size());
        return msg;
    }

private:
    zmq::context_t ctx_;
    zmq::socket_t sock_;
};
}  // namespace qualisys
#endif