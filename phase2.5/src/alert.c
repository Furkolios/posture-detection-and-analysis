#include <stdint.h>
#include "alert.h"
#include "hal_buzzer.h"

/* All cadences are expressed in main-loop ticks. With the default
 * 50 ms loop, a tick is 50 ms. */

/* MILD_SLOUCH: 100 ms on / 400 ms off, repeating. */
#define MILD_BEEP_PERIOD_TICKS       10u
#define MILD_BEEP_ON_TICKS            2u

/* LEAN_FORWARD / LEAN_BACK: short double beep — two short pulses
 * close together, then a longer gap. Ticks within a 12-tick (600 ms)
 * cycle: on, off, on, off, off, off, off, off, off, off, off, off. */
#define LEAN_PERIOD_TICKS            12u

/* LATERAL_TILT: three short pulses then a longer gap. 18-tick (900 ms)
 * cycle, pulses at ticks {0,1}, {3,4}, {6,7}, silent the rest. */
#define LATERAL_PERIOD_TICKS         18u

/* --- Module state -------------------------------------------------------- */
static uint32_t s_tick = 0u;

void alert_init(void) {
    buzzer_init();
    buzzer_off();
    s_tick = 0u;
}

/* Helper: should the lean cadence be on at the given phase? */
static int lean_phase_on(uint32_t phase) {
    return (phase == 0u || phase == 2u) ? 1 : 0;
}

/* Helper: should the lateral cadence be on at the given phase? */
static int lateral_phase_on(uint32_t phase) {
    return (phase == 0u || phase == 1u ||
            phase == 3u || phase == 4u ||
            phase == 6u || phase == 7u) ? 1 : 0;
}

void alert_update(PostureState state) {
    s_tick++;
    switch (state) {
        case POSTURE_GOOD:
            buzzer_off();
            break;

        case POSTURE_MILD_SLOUCH: {
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

        case POSTURE_LEAN_FORWARD:
        case POSTURE_LEAN_BACK: {
            uint32_t phase = s_tick % LEAN_PERIOD_TICKS;
            if (lean_phase_on(phase)) {
                buzzer_on();
            } else {
                buzzer_off();
            }
            break;
        }

        case POSTURE_LATERAL_TILT: {
            uint32_t phase = s_tick % LATERAL_PERIOD_TICKS;
            if (lateral_phase_on(phase)) {
                buzzer_on();
            } else {
                buzzer_off();
            }
            break;
        }

        default:
            /* Unknown state: fail safe to off. */
            buzzer_off();
            break;
    }
}
