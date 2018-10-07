#ifndef TRANSMITHANDLER_HPP_
#define TRANSMITHANDLER_HPP_

#include <stdint.h>
#include <cstdlib>
#include "Encoder.hpp"

struct TransmitHandler
{
	TransmitHandler(void (*f)(const uint8_t *, uint32_t));
	TransmitHandler() { sendFn = NULL; }
	Encoder e;									// encoder class
	void (*sendFn)(const uint8_t *, uint32_t);  // ptr to actual HW send function
};

void transmitPacket(const void *_buf, int buf_len, void *user);

#endif /* TransmitHandler.hpp */
