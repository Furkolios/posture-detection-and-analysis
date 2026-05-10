/*
 * hal_time_msp432.c — SysTick-based millisecond timer.
 *
 * Strategy: configure SysTick to fire every 1 ms and increment a
 * volatile counter in the ISR. time_ms() returns the counter; sleep_ms
 * busy-waits until the counter advances by the requested amount.
 *
 * Clock assumption: MCLK = 48 MHz (the default after the MSP432 boot
 * sequence configures the DCO and switches MCLK). SysTick is clocked
 * by MCLK, so reload value = MCLK_HZ / 1000 - 1.
 *
 * Wraparound: the counter is uint32_t. At 1 ms tick, it wraps every
 * ~49.7 days. The (now - start) subtraction below is wrap-safe in the
 * usual two's-complement-modular-arithmetic sense — sleep durations
 * up to ~24.8 days work even across a wrap.
 */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "hal_time.h"

#define MCLK_HZ                48000000u
#define SYSTICK_RELOAD_VALUE   ((MCLK_HZ / 1000u) - 1u)

static volatile uint32_t s_systick_ms = 0u;

/* SysTick ISR. The vector is named "SysTick_Handler" in TI's GCC
 * startup file; matching the name registers our handler. */
void SysTick_Handler(void);
void SysTick_Handler(void) {
    s_systick_ms++;
}

/* Internal init — called from main() before any sleep_ms or time_ms.
 * Exposed via a separate function (not part of hal_time.h) so the host
 * build doesn't need it. */
void hal_time_msp432_init(void);
void hal_time_msp432_init(void) {
    SysTick_disableModule();
    SysTick_setPeriod(SYSTICK_RELOAD_VALUE + 1u);  /* DriverLib takes period, not reload */
    SysTick_enableInterrupt();
    SysTick_enableModule();
    Interrupt_enableMaster();
}

uint32_t time_ms(void) {
    return s_systick_ms;
}

void sleep_ms(uint32_t ms) {
    uint32_t start = s_systick_ms;
    /* Wrap-safe: the subtraction is done in unsigned 32-bit, so it
     * gives the correct elapsed value even when s_systick_ms has
     * wrapped past start. */
    while ((uint32_t)(s_systick_ms - start) < ms) {
        /* WFI puts the core to sleep until the next interrupt (which
         * will be the next SysTick or an I2C/UART event). Saves power
         * vs a tight spin. The guard lets this file pass a host-side
         * syntax check on a non-ARM compiler. */
#if defined(__arm__) || defined(__thumb__)
        __asm volatile ("wfi");
#endif
    }
}
