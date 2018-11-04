#ifndef MAAV_DEBUG_HPP
#define MAAV_DEBUG_HPP
/**
 * Variadic macro for printing debug info when environment variable MAAV_DEBUG is defined
 *
 * Adapted from ZCM
 *
 * Author: Romario Pashollari (rpash@umich.edu)
 */

#include <stdio.h>
#include <stdlib.h>

extern bool MAAV_DEBUG_ENABLED;
void maav_debug_lock();
void maav_debug_unlock();

#define MAAV_DEBUG(...)                       \
    do                                        \
    {                                         \
        if (MAAV_DEBUG_ENABLED)               \
        {                                     \
            maav_debug_lock();                \
            fprintf(stderr, "[MAAV DEBUG] "); \
            fprintf(stderr, __VA_ARGS__);     \
            fprintf(stderr, "\n");            \
            fflush(stderr);                   \
            maav_debug_unlock();              \
        }                                     \
    } while (0)
#endif
