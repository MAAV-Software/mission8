#ifndef MAAV_SERIAL_HPP
#define MAAV_SERIAL_HPP

#include <sys/types.h>
#include <termios.h>
#include "Log.hpp"

namespace maav
{
/**
 * @brief Dual blocking/non-blocking serial I/O.
 *
 * @details Blocking functions are send() and receive(). Non-blocking users
 * should register a callback and call process() in a thread, and use send().
 */
class SerialTTY
{
   public:
	SerialTTY();

	explicit SerialTTY(const char *fname);

	~SerialTTY();

	/**
	 * @brief Send characters
	 * @param buffer data to send
	 * @param len Length of buffer
	 */
	void send(const char *buffer, size_t len);

	/**
	 * @brief Open a serial port
	 * @param port Port to open
	 * @return True on success, false otherwise
	 */
	void connect(const char *port);

	/**
	 * @brief Close the currently open port
	 * @return True on success or if already closed, false otherwise
	 */
	bool disconnect() noexcept;

	/**
	 * @brief Checks if connection is open
	 * @return True if open, false otherwise
	 */
	bool isConnected() const;

	/**
	 * @brief Performs a blocking read into buffer
	 * @param buffer Read characters will be stored here
	 * @param bufferLength length of buffer
	 * @return The amount of data read
	 */
	size_t receive(char *buffer, size_t bufferLength);

	/**
	 * @brief Receiving thread
	 */
	void process();

	/**
	 * @brief Stop process()ing stuff.
	 */
	void stop();

	void registerCallback(void (*func)(const char *, size_t));

   private:
	void (*cb)(const char *, size_t);
	bool connected;
	maav::Log::Logger log;
	bool running;
	struct termios tio;
	int tty;
};
}

#endif  // MAAV_SERIAL_HPP
