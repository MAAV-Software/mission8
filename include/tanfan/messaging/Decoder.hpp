#ifndef DECODER_HPP_
#define DECODER_HPP_

#include <stdint.h>


// states for data_frame_push_byte
typedef enum frame_state {READY, LEN_1, LEN_2, LEN_1_ESCP, LEN_2_ESCP, READ, 
						  READ_ESCP, CHECKSUM, CHECKSUM_ESCP, DONE, START_ERR, 
						  CHECKSUM_ERR, DATA_ERR} frameState;

class Decoder
{
public:
	// MODIFIES: *this
	// EFFECTS:  Tells the decoder that datum is the next byte in the
	//           received packet
	bool push(uint8_t datum);

	// EFFECTS:  Returns true if a full packet has been decoded. If a full
	//           packet has been decoded, it may be accessed using
	//           packetData() and packetSize().
	bool isDone() const { return state == DONE; }

	// EFFECTS:  Returns true if the Decoder is ready for the first byte of
	//           a packet to be received
	bool isReady() const { return state == READY; }

	// EFFECTS:  Returns true if an error has occured
	bool isError() const { return state == START_ERR; }

	// MODIFIES: *this
	// EFFECTS:  Resets the decoding. All decoded data so far will be lost,
	//           all errors will be cleared, the Decoder will be just like
	//           it was just constructed
	void reset() { at = size = 0; state = READY; }

	// REQUIRES: The decoding is done. Check with isDone().
	// EFFECTS:  Returns a pointer to the decoded packet data. If a full
	//           packet has not been decoded, the pointer is invalid and
	//           should NOT be used. Check whether a full packet has been
	//           decoded using isDone()
	const uint8_t* packetData() { return buffer; }

	// REQUIRES: The decoding is done. Check with isDone().
	// EFFECTS:  Returns the size of the packet. Note that this size is
	//           reflective of the size of the fully decoded packet! If
	//           you access packetData() before a packet is fully decoded,
	//           there WILL NOT be packetSize() bytes in it.
	uint32_t packetDataSize() const { return size; }

	// Creates new decoder.
	Decoder();

	// Decodes the information in raw and assigns it into buffer.
	Decoder& operator=(const uint8_t* raw);

private:
	uint32_t at;
	uint32_t size;
	uint8_t buffer[1024];
	frameState state;
};


#endif /* Decoder.hpp */
