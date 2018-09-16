#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include <stdint.h>


class Encoder
{
public:
	// REQUIRES: message is at least as large as messageSize and no larger
	//           than 120 bytes.
	// EFFECTS:  Encodes the first messageSize bytes of the message.
	uint32_t encode(const uint8_t* message, uint32_t messageSize);

	// REQUIRES: Stuff has been encoded
	// EFFECTS:  Returns a pointer to the encoded packet.
	const uint8_t* packet() const { return pkt; }

	// REQUIRES: Stuff has been encoded
	// EFFECTS:  Returns the size of the encoded packet.
	uint32_t packetSize() const { return pktSize; }

	//Encoder(); // Does not need one
private:
	uint32_t pktSize;
	uint8_t pkt[256];
};


#endif /* Encoder.hpp */
