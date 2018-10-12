#include "tanfan/nav/data_link.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

char transmit_user[256];

// char* data_frame_state_names[] = { "READY", "LEN_1", "LEN_2", "LEN_1_ESCP", "LEN_2_ESCP", "READ",
// "READ_ESCP", "CHECKSUM", "CHECKSUM_ESCP", "DONE", "START_ERR", "CHECKSUM_ERR", "DATA_ERR" };
char* feedback_channel_name = CHANNEL_FEEDBACK;
char* position_channel_name = CHANNEL_POSITION;
char* target_channel_name = CHANNEL_TARGET;
char* tuning_channel_name = CHANNEL_TUNING;

static inline void data_link_branchless_assemble_byte(uint8_t* pkt, size_t* pkt_i,
													  uint8_t data_byte);

// static inline uint8_t data_link_branchless_decode_byte(uint8_t* pkt, size_t* pkt_i);

data_frame_t* data_frame_create(uint16_t size)
{
	data_frame_t* frame = (data_frame_t*)malloc(sizeof(data_frame_t));
	frame->buffer = (uint8_t*)malloc(sizeof(uint8_t) * size);
	frame->size = 0;
	frame->index = 0;
	// frame->state = READY;

	return frame;
}

void data_frame_destroy(data_frame_t* frame)
{
	free(frame->buffer);
	free(frame);
}

void data_frame_clear(data_frame_t* frame)
{
	frame->size = 0;
	frame->index = 0;
	// frame->state = READY;
}
/*
// state machine to process bytes in the data link layer
// TODO: try to make branchless
// TODO: check size
bool data_frame_push_byte(data_frame_t* frame, uint8_t byte) {
	switch (frame->state) {
		case READY:
			if (byte != DATA_FRAME_START_DELIMITER) {
				frame->state = START_ERR;
				return false;
			} else {
				frame->state = LEN_1;
			}
			break;
		case LEN_1:
			if (byte == DATA_FRAME_ESCAPE_CHAR) {
				frame->state = LEN_1_ESCP;
			} else {
				frame->size = (uint16_t) byte << 8;
				frame->state = LEN_2;
			}
			break;
		case LEN_2:
			if (byte == DATA_FRAME_ESCAPE_CHAR) {
				frame->state = LEN_2_ESCP;
			} else {
				frame->size |= byte & 0xFF;
				frame->state = READ;
			}
			if (frame->size == 0) {
				frame->state = DONE;
			}
			break;
		case LEN_1_ESCP:
			frame->size = (uint16_t) (byte ^ DATA_FRAME_XOR) << 8;
			frame->state = LEN_2;
			break;
		case LEN_2_ESCP:
			frame->size |= (byte ^ DATA_FRAME_XOR) << 8;
			frame->state = READ;
			if (frame->size == 0) {
				frame->state = DONE;
			}
			break;
		case READ:
			if (byte == DATA_FRAME_ESCAPE_CHAR) {
				frame->state = READ_ESCP;
			} else {
				(frame->buffer)[frame->index] = byte;
				frame->checksum += byte;
				frame->index++;
				if (frame->index == frame->size) {
					frame->state = CHECKSUM;
				}
			}
			break;
		case READ_ESCP:
			(frame->buffer)[frame->index] = byte ^ DATA_FRAME_XOR;
			frame->checksum += byte ^ DATA_FRAME_XOR;
			frame->index++;
			if (frame->index == frame->size) {
				frame->state = CHECKSUM;
			} else {
				frame->state = READ;
			}
			break;
		case CHECKSUM:
			if (byte == DATA_FRAME_ESCAPE_CHAR) {
				frame->state = CHECKSUM_ESCP;
			} else {
				frame->state = DONE;
			}
			break;
		case CHECKSUM_ESCP:
			frame->state = DONE;
			break;
		default:
			return false;
	}
	return true;
}
*/
uint16_t data_link_assemble_packet(uint8_t* msg, uint8_t* pkt, uint16_t size)
{
	size_t msg_i = 0, pkt_i = 0;
	pkt[pkt_i++] = DATA_FRAME_START_DELIMITER;

	data_link_branchless_assemble_byte(pkt, &pkt_i, size >> 8);
	pkt_i++;
	data_link_branchless_assemble_byte(pkt, &pkt_i, size & 0xFF);
	pkt_i++;

	uint8_t checksum = 0;
	while (msg_i < size)
	{
		data_link_branchless_assemble_byte(pkt, &pkt_i, msg[msg_i]);
		checksum += msg[msg_i++];
		pkt_i++;
	}

	checksum = 0xFF - checksum;
	data_link_branchless_assemble_byte(pkt, &pkt_i, checksum);
	return pkt_i + 1;
}
/*
bool data_link_decode_packet(uint8_t* msg, uint8_t* pkt, uint16_t* size) {
	size_t msg_i = 0, pkt_i = 0;
	if (pkt[pkt_i++] != DATA_FRAME_START_DELIMITER) {
		return false;
	}

	uint8_t size_upper = data_link_branchless_decode_byte(pkt, &pkt_i);
	pkt_i++;
	uint8_t size_lower = data_link_branchless_decode_byte(pkt, &pkt_i);
	pkt_i++;
	*size = (uint16_t) (size_upper << 8) | size_lower;

	uint8_t checksum = 0;
	while (msg_i < *size) {
		msg[msg_i] = data_link_branchless_decode_byte(pkt, &pkt_i);
		checksum += msg[msg_i++];
		pkt_i++;
	}

	checksum = 0xFF - checksum;
	uint8_t pkt_checksum = data_link_branchless_decode_byte(pkt, &pkt_i);
	if (checksum != pkt_checksum) {
		return true;//Changed from false for testing
	}

	return true;
}
*/
static inline void data_link_branchless_assemble_byte(uint8_t* pkt, size_t* pkt_i,
													  uint8_t data_byte)
{
	uint8_t is_special_char =
		(data_byte == DATA_FRAME_START_DELIMITER) + (data_byte == DATA_FRAME_ESCAPE_CHAR);
	uint8_t xor_val = is_special_char << 5;
	pkt[*pkt_i] = DATA_FRAME_ESCAPE_CHAR;
	pkt[*pkt_i + is_special_char] = data_byte ^ xor_val;
	*pkt_i += is_special_char;
}
/*
static inline uint8_t data_link_branchless_decode_byte(uint8_t* pkt, size_t* pkt_i) {
	uint8_t is_delimited = (pkt[*pkt_i] == DATA_FRAME_ESCAPE_CHAR);
	uint8_t xor_val = is_delimited << 5;
	*pkt_i += is_delimited;
	return (pkt[*pkt_i] ^ xor_val);
}
*/
