#include "tanfan/messaging/TransmitHandler.hpp"
#include <stdint.h>
#include "tanfan/messaging/Encoder.hpp"

TransmitHandler::TransmitHandler(void (*f)(const uint8_t *, uint32_t)) : sendFn(f) {}
void transmitPacket(const void *_buf, int buf_len, void *user)
{
    TransmitHandler *t = (TransmitHandler *)user;
    t->e.encode((uint8_t *)_buf, buf_len);
    t->sendFn(t->e.packet(), t->e.packetSize());
}
