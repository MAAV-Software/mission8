#include <cstdint>

#include "common/utils/MsgValidator.hpp"

MsgValidator::MsgValidator() : prevTime(0) {}
bool MsgValidator::operator()(int64_t utime)
{
    if (utime > prevTime)
    {
        prevTime = utime;
        return true;
    }
    return false;
}
