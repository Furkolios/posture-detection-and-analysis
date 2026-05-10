#ifndef HAL_TIME_H
#define HAL_TIME_H

#include <stdint.h>

/* Milliseconds since boot (or since process start, on host). */
uint32_t time_ms(void);

/* Block the calling context for the given number of milliseconds. */
void sleep_ms(uint32_t ms);

#endif /* HAL_TIME_H */
