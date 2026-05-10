/*
 * hal_buzzer_msp432.c — Active piezo buzzer driver for MSP432.
 *
 * Pin choice: P2.4 (LaunchPad header J4.31). Any free GPIO works; if
 * you change the pin, update BUZZER_PORT and BUZZER_PIN below AND the
 * wiring table in README_teammates.md.
 *
 * The buzzer is assumed to be ACTIVE (has its own oscillator) — drive
 * the pin high to make sound, low to silence. If you swap in a passive
 * buzzer, this file needs a Timer_A PWM rewrite; the rest of the
 * firmware is unaffected.
 */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "hal_buzzer.h"

#define BUZZER_PORT   GPIO_PORT_P2
#define BUZZER_PIN    GPIO_PIN4

void buzzer_init(void) {
    GPIO_setAsOutputPin(BUZZER_PORT, BUZZER_PIN);
    GPIO_setOutputLowOnPin(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_on(void) {
    GPIO_setOutputHighOnPin(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_off(void) {
    GPIO_setOutputLowOnPin(BUZZER_PORT, BUZZER_PIN);
}
