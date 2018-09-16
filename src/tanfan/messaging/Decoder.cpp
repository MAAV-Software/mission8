#include <stdint.h>
#include "tanfan/messaging/Decoder.hpp"
#include "tanfan/messaging/DataLinkDefines.hpp"


static inline uint8_t data_link_branchless_decode_byte(const uint8_t* pkt, uint32_t* pkt_i);

Decoder::Decoder() : at(0), size(0), state(READY) {}

bool Decoder::push(uint8_t byte)
{
    switch (state)
	{
		case READY:
			if (byte != DATA_FRAME_START_DELIMITER)
			{
				state = START_ERR;
				return false;
			}
			else state = LEN_1;
			break;
		case LEN_1:
			if (byte == DATA_FRAME_ESCAPE_CHAR) state = LEN_1_ESCP;
			else
			{
				size = (uint32_t) byte << 8;
				state = LEN_2;
			}
			break;
		case LEN_2:
			if (byte == DATA_FRAME_ESCAPE_CHAR) state = LEN_2_ESCP;
			else
			{
				size |= byte & 0xFF;
				state = READ;
			}
			if (size == 0) state = DONE;
			break;
		case LEN_1_ESCP:
			size = (uint32_t) (byte ^ DATA_FRAME_XOR) << 8;
			state = LEN_2;
			break;
		case LEN_2_ESCP:
			size |= (byte ^ DATA_FRAME_XOR) << 8;
			state = READ;
			if (size == 0) state = DONE;
			break;
		case READ:
			if (byte == DATA_FRAME_ESCAPE_CHAR) state = READ_ESCP;
			else
			{
				(buffer)[at] = byte;
				at++;
				if (at == size) state = CHECKSUM;
			}
			break;
		case READ_ESCP:
			(buffer)[at] = byte ^ DATA_FRAME_XOR;
			at++;
			if (at == size) state = CHECKSUM;
			else 			state = READ;
			break;
		case CHECKSUM:		state = DONE; break;
		case CHECKSUM_ESCP:	state = DONE; break;
		default:			return false; break;
	}

	return true;
}

static inline uint8_t data_link_branchless_decode_byte(const uint8_t* pkt, uint32_t* pkt_i)
{
    uint8_t is_delimited = (pkt[*pkt_i] == DATA_FRAME_ESCAPE_CHAR);
    uint8_t xor_val = (uint8_t)(is_delimited << 5);
    *pkt_i += is_delimited;
    return (pkt[*pkt_i] ^ xor_val);
}

Decoder& Decoder::operator=(const uint8_t* pkt)
{
    uint32_t msg_i = 0, pkt_i = 0;
    if (pkt[pkt_i++] != DATA_FRAME_START_DELIMITER)
	{
        state = START_ERR;
        return *this;
    }

    uint8_t size_upper = data_link_branchless_decode_byte(pkt, &pkt_i);
    pkt_i++;
    uint8_t size_lower = data_link_branchless_decode_byte(pkt, &pkt_i);
    pkt_i++;
    size = (uint32_t) (size_upper << 8) | size_lower;

    uint8_t checksum = 0;
    while (msg_i < size)
	{
        buffer[msg_i] = data_link_branchless_decode_byte(pkt, &pkt_i);
        checksum = (uint8_t)(checksum + buffer[msg_i++]);
        pkt_i++;
    }

    state = DONE;
    return *this;
}
