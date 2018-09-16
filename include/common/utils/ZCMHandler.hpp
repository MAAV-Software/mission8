#ifndef ZCMHANDLER_HPP
#define ZCMHANDLER_HPP

#include <zcm/zcm-cpp.hpp>
#include <string>
#include <queue>
#include <mutex>

/**
 * @brief ZCM Message Handler
 * @details Templated class that implements an ZCM message callback and queue of
 *			messages. It is also thread-safe (using C++11 STL Multi-Threading
 *			implementation).
 * @author Sajan Patel (sajanptl@umich.edu)
 */
template <typename T>
class ZCMHandler
{
public:
	/**
	 * @brief Constructs a default MessageHander with an empty message queue.
	 */
	ZCMHandler() = default;

	/**
	 * @brief ZCM Message Callback
	 * @details This function gets called upon the arrival of an ZCM message as
	 *			handled by ZCM::handle() or LMC::handleTimeout(). It pushes the
	 *			received message object to the back of the queue. This function
	 *			is to be registered as the callback for this message type when
	 *			subscribing to the message and channel as follows (assume
	 *			ZCM object "zcm" and ZCMHandler "mh" for message type "T"
	 *			are already instantiated):
	 *
	 *			zcm.subscribe("channel", &ZCMHandler<T>::recv, &mh);
	 *
	 *			The arguments to this function satisfy the function pointer
	 *			interface that ZCM::subscribe() requires (see ZCM documentation).
	 * @param rbuf		ZCM reciever buffer
	 * @param channel	ZCM channel being subsribed to for this message
	 * @param msg		pointer to the newly received ZCM message
	 */
	void recv(const zcm::ReceiveBuffer*,
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

/**
 * @brief ZCM Message Handler that just stores the latest message
 * @details Templated class that implements an ZCM message callback for the latest
 *			message. It is also thread-safe (using C++11 STL Multi-Threading
 *			implementation).
 * @author neckardt and sajanptl
 */
template <typename T>
class ZCMSingleHandler
{
public:
	/**
	 * @brief Constructs a default MessageHander with an empty message queue.
	 */
	ZCMSingleHandler( T msg_in)
	{
		msg_ = msg_in;
	}

	/**
	 * @brief ZCM Message Callback
	 * @details This function gets called upon the arrival of an ZCM message as
	 *			handled by ZCM::handle() or LMC::handleTimeout(). It pushes the
	 *			received message object to the back of the queue. This function
	 *			is to be registered as the callback for this message type when
	 *			subscribing to the message and channel as follows (assume
	 *			ZCM object "zcm" and ZCMHandler "mh" for message type "T"
	 *			are already instantiated):
	 *
	 *			zcm.subscribe("channel", &ZCMHandler<T>::recv, &mh);
	 *
	 *			The arguments to this function satisfy the function pointer
	 *			interface that ZCM::subscribe() requires (see ZCM documentation).
	 * @param rbuf		ZCM reciever buffer
	 * @param channel	ZCM channel being subsribed to for this message
	 * @param msg		pointer to the newly received ZCM message
	 */
	void recv(const zcm::ReceiveBuffer*,
			  const std::string&,
			  const T* msg)
	{
		std::lock_guard<std::mutex> lck(mtx);
		msg_ = *msg;
	}
 	/**
	 * @brief Returns the message at the front of the queue.
	 */
	T msg()
	{
		std::lock_guard<std::mutex> lck(mtx);
		return msg_;
	}
private:
	T msg_; 			///< latest message
	std::mutex mtx;		///< mutex for locking the queue (use with std::lock_guard)
};

#endif
