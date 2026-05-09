#define _POSIX_C_SOURCE 199309L
#include <time.h>
#include "hal_time.h"

/* Host stub for time and sleep. Uses CLOCK_MONOTONIC so it is
 * unaffected by wall-clock adjustments. */

static uint32_t boot_offset_ms(void) {
    static int initialised = 0;
    static uint32_t offset = 0u;
    if (!initialised) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        offset = (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
        initialised = 1;
    }
    return offset;
}

uint32_t time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint32_t now = (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
    return now - boot_offset_ms();
}

void sleep_ms(uint32_t ms) {
    struct timespec req;
    req.tv_sec  = (time_t)(ms / 1000u);
    req.tv_nsec = (long)((ms % 1000u) * 1000000L);
    nanosleep(&req, NULL);
}
