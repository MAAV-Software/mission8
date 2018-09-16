#include <stdint.h>
#include "tanfan/messaging/DataLinkDefines.hpp"

// Data frame special values
const uint8_t DATA_FRAME_START_DELIMITER = 0x7E;
const uint8_t DATA_FRAME_ESCAPE_CHAR = 0x7D;
const uint8_t DATA_FRAME_XOR = 0x20;
