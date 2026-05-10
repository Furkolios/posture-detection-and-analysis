/*
 * hal_platform_host.c — No-op platform init for the host build.
 *
 * The MSP432 build needs clock and SysTick configuration before any
 * peripheral is used. The host build needs nothing — POSIX takes care
 * of timing and there's no clock tree to configure. We expose the
 * same symbol so main.c can call it unconditionally on both targets.
 */

void hal_platform_init(void);
void hal_platform_init(void) {
    /* Nothing to do on the host. */
}
