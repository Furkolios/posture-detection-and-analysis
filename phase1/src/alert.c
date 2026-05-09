#include <stdint.h>
#include "alert.h"
#include "hal_buzzer.h"

/* Number of alert_update() ticks in one full short-beep cycle when in
 * MILD_SLOUCH. Tweak with the main loop period to taste. With a 50 ms
 * loop, period of 10 = 500 ms cycle. */
#define MILD_BEEP_PERIOD_TICKS  10u

/* For how many ticks of the cycle the buzzer is on. Must be < period. */
#define MILD_BEEP_ON_TICKS       2u

/* --- Module state -------------------------------------------------------- *
 * Tick counter is the only piece of state we need. Marked static so it's
 * private to this module. */
static uint32_t s_tick = 0u;

void alert_init(void) {
    buzzer_init();
    buzzer_off();
    s_tick = 0u;
}

void alert_update(PostureState state) {
    s_tick++;
    switch (state) {
        case POSTURE_GOOD:
            buzzer_off();
            break;

        case POSTURE_MILD_SLOUCH: {
            /* Short periodic beep: on for the first MILD_BEEP_ON_TICKS
             * of each MILD_BEEP_PERIOD_TICKS-tick cycle, off for the
             * rest. */
            uint32_t phase = s_tick % MILD_BEEP_PERIOD_TICKS;
            if (phase < MILD_BEEP_ON_TICKS) {
                buzzer_on();
            } else {
                buzzer_off();
            }
            break;
        }

        case POSTURE_FULL_SLOUCH:
            buzzer_on();
            break;

        default:
            /* Defensive: an unknown state should never silently ignore
             * — fail safe to off. */
            buzzer_off();
            break;
    }
}
