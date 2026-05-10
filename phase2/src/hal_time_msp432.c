#include "hal_time.h"

/* MSP432 time HAL placeholders. Phase 2 will back these with SysTick.
 * Strategy: enable SysTick at 1 ms tick rate, increment a static
 * volatile uint32_t in the ISR, return it from time_ms(). sleep_ms
 * will busy-wait against time_ms() — fine for now, can move to LPM
 * with timer wakeup later if power matters. */

uint32_t time_ms(void) {
    /* TODO Phase 2: return s_systick_ms; */
    return 0u;
}

void sleep_ms(uint32_t ms) {
    (void)ms;
    /* TODO Phase 2:
     *   uint32_t start = time_ms();
     *   while ((time_ms() - start) < ms) { __no_operation(); }
     */
}
