#ifndef _ARDUINO_H_
#define _ARDUINO_H_

#include <stdint.h>
#include <time.h>

static inline uint32_t millis()
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return static_cast<uint32_t>(tp.tv_sec * 1000) + static_cast<uint32_t>(tp.tv_nsec / 1000000);
}

#endif
