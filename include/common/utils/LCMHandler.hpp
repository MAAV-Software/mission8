#ifndef LCMHANDLER_HPP
#define LCMHANDLER_HPP

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <queue>
#include <mutex>

/**
 * @brief LCM Message Handler
 * @details Templated class that implements an LCM message callback and queue of
 *			messages. It is also thread-safe (using C++11 STL Multi-Threading
 *			implementation).
 * @author Sajan Patel (sajanptl@umich.edu)
 */
template <typename T>
class LCMHandler
{
public:
	/**
	 * @brief Constructs a default MessageHander with an empty message queue.
	 */
	LCMHandler() = default;

	/**
	 * @brief LCM Message Callback
	 * @details This function gets called upon the arrival of an LCM message as
	 *			handled by LCM::handle() or LMC::handleTimeout(). It pushes the
	 *			received message object to the back of the queue. This function
	 *			is to be registered as the callback for this message type when
	 *			subscribing to the message and channel as follows (assume
	 *			LCM object "lcm" and LCMHandler "mh" for message type "T"
     *			are already instantiated):
	 *
	 *			lcm.subscribe("channel", &LCMHandler<T>::recv, &mh);
	 *
	 *			The arguments to this function satisfy the function pointer
	 *			interface that LCM::subscribe() requires (see LCM documentation).
	 * @param rbuf		LCM reciever buffer
	 * @param channel	LCM channel being subsribed to for this message
	 * @param msg		pointer to the newly received LCM message
	 */
	void recv(const lcm::ReceiveBuffer*,
			  const std::string&,
			  const T* msg)
    {
        std::lock_guard<std::mutex> lck(mtx);
        msgs.push(*msg);
    }

 	/**
     * @brief Returns the message at the front of the queue.
	 */
    T msg()
    {
        std::lock_guard<std::mutex> lck(mtx);
        return msgs.front();
    }

	/**
	 * @bief Returns true if the queue is not empty; false otherwise.
	 */
    bool ready()
    {
        std::lock_guard<std::mutex> lck(mtx);
        return !msgs.empty();
    }

	/**
	 * @brief Pops the message at the front of the queue.
	 */
    void pop()
    {
        std::lock_guard<std::mutex> lck(mtx);
        if (!msgs.empty()) msgs.pop();
    }

	/**
	 * @brief Returns the size of the message queue.
	 */
	size_t size()
	{
		std::lock_guard<std::mutex> lck(mtx);
		return msgs.size();
	}

private:
    std::queue<T> msgs; ///< message queue
    std::mutex mtx;		///< mutex for locking the queue (use with std::lock_guard)
};

#endif
