#include "common/utils/debug.hpp"

bool MAAV_DEBUG_ENABLED;

#include <mutex>
static std::mutex mtx;

void maav_debug_lock() { mtx.lock(); }
void maav_debug_unlock() { mtx.unlock(); }
// This code runs before the main method and
// checks if the environment variable has been set
struct BeforeMain
{
    BeforeMain() { MAAV_DEBUG_ENABLED = (nullptr != getenv("MAAV_DEBUG")); }
};
static BeforeMain run;
