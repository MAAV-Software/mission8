#include "tanfan/messaging/Encoder.hpp"
#include <stdint.h>
#include "tanfan/messaging/DataLinkDefines.hpp"

// A helper function for Encoding
// REQUIRES: pkt is at least of size pkt_i + 1.
// MODIFIES: pkt, pkt_i
// EFFECTS:  Encodes data_byte into packet pkt at position pkt_i and advances
//           pkt_i to the next position, all without branching
static inline void data_link_branchless_assemble_byte(uint8_t* pkt, uint32_t* pkt_i,
													  uint8_t data_byte);

uint32_t Encoder::encode(const uint8_t* msg, uint32_t size)
{
	uint32_t msg_i = 0, pkt_i = 0;
	pkt[pkt_i++] = DATA_FRAME_START_DELIMITER;

	data_link_branchless_assemble_byte(pkt, &pkt_i, (uint8_t)(size >> 8));
	pkt_i++;
	data_link_branchless_assemble_byte(pkt, &pkt_i, size & 0xFF);
	pkt_i++;

	uint8_t checksum = 0;
	while (msg_i < size)
	{
		data_link_branchless_assemble_byte(pkt, &pkt_i, msg[msg_i]);
		checksum = (uint8_t)(checksum + msg[msg_i++]);
		pkt_i++;
	}

	checksum = (uint8_t)(0xFF - checksum);
	data_link_branchless_assemble_byte(pkt, &pkt_i, checksum);
	pktSize = pkt_i + 1;

	return pktSize;
}

static inline void data_link_branchless_assemble_byte(uint8_t* pkt, uint32_t* pkt_i,
													  uint8_t data_byte)
{
	uint8_t is_special_char = (uint8_t)((data_byte == DATA_FRAME_START_DELIMITER) +
										(data_byte == DATA_FRAME_ESCAPE_CHAR));
	uint8_t xor_val = (uint8_t)(is_special_char << 5);

	pkt[*pkt_i] = DATA_FRAME_ESCAPE_CHAR;

	pkt[*pkt_i + is_special_char] = data_byte ^ xor_val;

	*pkt_i += is_special_char;
}
