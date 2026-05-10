#include "hal_buzzer.h"

/* MSP432 buzzer driver. Phase 1 keeps these as empty placeholders so
 * the firmware target compiles even on hosts without the MSP432
 * DriverLib. Phase 2 (Ahmet Faruk) will fill them in with real
 * GPIO_setOutputHighOnPin / setOutputLowOnPin calls.
 *
 * Pin choice (tentative): P2.4 — easy to wire on the LaunchPad. */

void buzzer_init(void) {
    /* TODO Phase 2:
     *   GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4);
     *   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
     */
}

void buzzer_on(void) {
    /* TODO Phase 2:
     *   GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);
     */
}

void buzzer_off(void) {
    /* TODO Phase 2:
     *   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
     */
}
