#ifndef MAAV_GCS_GLIB_ZCM_HPP
#define MAAV_GCS_GLIB_ZCM_HPP

#include <glibmm/main.h>
#include <sigc++/sigc++.h>
#include <zcm/zcm-cpp.hpp>
#include "common/utils/debug.hpp"

#include <string>

namespace maav
{
namespace gcs
{
/**
 * @brief A version of the ZCM class with Glib integration
 *
 * @details This ZCM subclass replaces the subscription capabilities of ZCM with
 * an interface which makes use of Glib signalling mechanisms to report events
 * using the Glib event loop.
 *
 * One of these can be constructed simply like a normal ZCM, and provides the
 * same publishing interface when constructed, with the exception that messages
 * are passed by reference instead of through a pointer:
 *
 * ```
 * GlibZCM zcm;
 * zcm.publish(some_channel_name, some_message);
 * ```
 *
 * In order to handle incoming messages, the nested class template Handler is
 * included which provides an RAII-enabled subscription to a channel and a
 * signal-based interface to provide a callback for handling messages:
 *
 * ```
 * {
 *     GlibZCM::Handler<SomeMessageType> hand{zcm, some_channel_name};
 *     hand.signal_message().connect([](const SomeMessageType& mess) {
 *         log_message(mess);
 *         do_stuff_with_message(mess);
 *     });
 *
 *     //the handler will automatically unsubscribe here
 * }
 * ```
 *
 * Messages can also be handled by deriving from a Handler type and overriding
 * its message-handling virtual function:
 *
 * ```
 * class MyHandler : GlibZCM::Handler<SomeMessageType>
 * {
 * protected:
 *     void on_message(const SomeMessageType& mess) override {
 *         do_message_stuff(mess);
 *     }
 * public:
 *     MyHandler(GlibZCM& zcm, const char* channel)
 *         : GlibZCM::Handler<SomeMessageType>{zcm, channel} {}
 * };
 * ```
 *
 * Which one of these two options should be used in a given piece of code
 * depends on the kind of functionality needed: separate Handlers and signals
 * are more flexible and can be used on multiple channels from the same place,
 * but overriding the virtual function can be much simpler.
 */
class GlibZCM : zcm::ZCM
{
	// the connection used to recieve events when input is available on the ZCM
	// FD
	sigc::connection zcm_connection;

	// the callback for that connection
	bool handle_input();

   public:
	/**
	 * @brief Creates an sets up a Glib-enabled ZCM connection
	 */
	explicit GlibZCM(const std::string& url);

	~GlibZCM();

	template <typename Message>
	void publish(const std::string& channel, const Message& msg)
	{
		MAAV_DEBUG("Publishing %s message on channel %s", msg.getTypeName(), channel.c_str());
		ZCM::publish(channel, &msg);
	}

	/**
	 * @brief A helper type template for subscribing to messages
	 * @param Message the message type to use
	 *
	 * @details See GlibZCM for usage details
	 */
	template <typename Message>
	class Handler
	{
		// the corresponding GlibZCM instance
		GlibZCM& zcm;

		// the subscription that this manages
		zcm::Subscription* subscription;

		// the signal used for handling messages by default
		sigc::signal<void, const Message&> message_signal;

		// the callback for invoking on_message when ZCM finds messages
		void handle_message(const zcm::ReceiveBuffer*, const std::string& channel,
							const Message* msg)
		{
			MAAV_DEBUG("ZCM found a message of type %s on channel %s", msg->getTypeName(),
					   channel.c_str());
			return on_message(*msg);
		}

	   protected:
		/**
		 * @brief The virtual function used for handling messages
		 * @param msg The message to handle
		 */
		virtual void on_message(const Message& msg) { message_signal.emit(msg); }
		// the channel used for this subscription
		const std::string channel;

	   public:
		/**
		 * @brief Creates a handler for handling messages from a given channel
		 * @param zcm_in The GlibZCM object to use the handle with
		 * @param channel The ZCM channel to subscribe to
		 */
		Handler(GlibZCM& zcm_in, const std::string& channel_in)
			: zcm{zcm_in},
			  subscription{zcm.subscribe(channel_in, &Handler::handle_message, this)},
			  channel{channel_in}
		{
			MAAV_DEBUG("GlibZCM set to handle messages on channel %s.", channel.c_str());
		}

		~Handler() { zcm.unsubscribe(subscription); }
		/**
		 * @brief Exposes this handler's signal
		 * @return The signal; connect to this to handle messages
		 */
		sigc::signal<void, const Message&>& signal_message() { return message_signal; }
		/**
		 * @brief Retreives the GlibZCM that this Handler belongs to
		 * @return The Handler's GlibZCM
		 */
		GlibZCM& get_zcm() const { return zcm; }
	};
};
}
}

#endif
